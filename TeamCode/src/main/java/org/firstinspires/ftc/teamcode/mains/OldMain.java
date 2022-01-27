package org.firstinspires.ftc.teamcode.mains;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;

// This annotates the class to tell the robot controller app that it is a TeleOp OpMode
@TeleOp(name="OldMain",group="")
@Disabled
public class OldMain extends LinearOpMode {
    // Pre-init
    private final Nikolaj robot = new Nikolaj(); // Library with robot config
    private final FieldCentric drive = new FieldCentric(); // Library with field centric drive functions
    private final FtcDashboard dashboard = FtcDashboard.getInstance(); // FtcDashboard instance
    private final TelemetryPacket packet = new TelemetryPacket(); // Telemetry packet to send to dashboard

    // This function is ran when you press init
    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap); // Initializes the config

        // Manual initialization of stuff because testing
        final CRServo intake = robot.getLSrv();
        final CRServo spinner = robot.getRSrv();

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu1");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = false;
        params.loggingTag = "LolNoThanks";
        imu.initialize(params);

        // Set up field centric driving
        drive.setUp(
            new DcMotor[]{robot.frMotor(), robot.rrMotor(), robot.rlMotor(), robot.flMotor()},
            new double[]{PI/4, 3*PI/4, 5*PI/4, 7*PI/4},
            imu,
            hardwareMap
        );

        // Lifter positions (min is actually how high it can go)
        int lifterMin = -8000;
        int lifterMax = 0;
        double lifterSpeed;
        boolean lifterMoving = false;

        int intaking = 0;
        double spinSpeed = .7;

        boolean slower = false;
        double slowerSpeed = .5;

        // Toast notification because I can
        AppUtil.getInstance().showToast(UILocation.BOTH, "Iteration 7.3-8.3");
        waitForStart(); // This function halts the program until you press start

        // Pre-run
        // Copies of gamepads to save their state from the previous cycle (you'll see the use of that later)
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        while (opModeIsActive()) {
            // TeleOp loop
            drive.gyro(); // Update angles in the field centric program
            // Make it drive
            drive.Drive(
                    !slower ? -gamepad1.left_stick_x : -gamepad1.left_stick_x * slowerSpeed,
                    !slower ? gamepad1.left_stick_y : gamepad1.left_stick_y * slowerSpeed,
                    !slower ? -gamepad1.right_stick_x * .5 : -gamepad1.right_stick_x * slowerSpeed * .5
            );
            
            if (gamepad1.back && !cp1.back) { drive.resetAngle(); } // Reset angle in field centric program

            // Controls the lifter
            lifterSpeed = gamepad2.left_stick_y;
            int curPos = robot.getLifter().getCurrentPosition();
            if (lifterMoving) {
                if (lifterSpeed > 0.1 || lifterSpeed < -0.1) { lifterMoving = false; lifterSpeed = (0); }
                if (curPos < -6653) { lifterSpeed = 1; }
                else if (curPos < -6652) { lifterMoving = false; lifterSpeed = 0; }
                else if (curPos > -6651) { lifterSpeed = (-1); }
            } else if (curPos > lifterMax - 2) { // If the lifter is near the maximum, don't let it go down more (because that's how the lifter works)
                lifterSpeed = clamp(-1, 0, lifterSpeed);
            } else if (curPos < lifterMin + 2) { // If the lifter is near the minimum, don't left it go up
                lifterSpeed = clamp(0, 1, lifterSpeed);
            }

            if (gamepad2.y && !cp2.y) {
                lifterMoving = !lifterMoving;
            }
            robot.getLifter().setPower(lifterSpeed); // Actually set the speed
            telemetry.addData("Lifter", curPos); // -6652

            // Intake control
            if (intaking == 0 && gamepad2.a && !cp2.a) {
                intake.setPower(spinSpeed);
                spinner.setPower(-spinSpeed);
                intaking = 1;
            } else if (intaking != 0 && ((gamepad2.a && !cp2.a) || (gamepad2.b && !cp2.b))) {
                intake.setPower(0);
                spinner.setPower(0);
                intaking = 0;
            } else if (intaking == 0 && gamepad2.b && !cp2.b) {
                intake.setPower(-spinSpeed + .1);
                spinner.setPower(spinSpeed - .1);
                intaking = -1;
            }

            // Stop intaking if we are intaking and the touch sensor is pressed
            if (intaking == 1 && robot.getTouch().isPressed()) {
                robot.getLSrv().setPower(0);
                robot.getRSrv().setPower(0);
                intaking = 0;
            }

            // Runs the servo to spin the duck thing
            if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                spinner.setPower(1);
                intake.setPower(0);
                intaking = 0;
            } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                intake.setPower(-1);
                spinner.setPower(0);
                intaking = 0;
            } else if (intaking == 0) {
                spinner.setPower(0);
                intake.setPower(0);
            }

            if (gamepad1.x && !cp1.x) {
                slower = !slower;
            }
            telemetry.addData("Slower", slower);

            telemetry.update();

            try {
                // Copy the gamepad state at the end of the cycle for use in the next cycle
                // This allows us to know what buttons have (not) changed
                cp1.copy(gamepad1);
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
        }
    }

    // Clamp function because there is no native clamp
    private double clamp(double min, double max, double value) {
        return Math.min(max, Math.max(value, min));
    }
}
