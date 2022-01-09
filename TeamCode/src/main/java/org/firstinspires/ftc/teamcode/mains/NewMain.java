package org.firstinspires.ftc.teamcode.mains;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;

@TeleOp(name="",group="")
public class NewMain extends LinearOpMode {
    // Pre-init
    private final Nikolaj robot = new Nikolaj(); // Library with robot config
    private final FieldCentric drive = new FieldCentric(); // Library with field centric drive functions
    private final FtcDashboard dashboard = FtcDashboard.getInstance(); // FtcDashboard instance
    private final TelemetryPacket packet = new TelemetryPacket(); // Telemetry packet to send to dashboard
    
    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap); // Initializes the config

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

        // Drive powers
        double lifterPower, lSrvPower, rSrvPower;
        double lifterPrevP, lSrvPrevP, rSrvPrevP;
        lifterPower = lSrvPower = rSrvPower = 0;
        lifterPrevP = lSrvPrevP = rSrvPrevP = 0;

        int armPos, armPrevP;
        armPos = armPrevP = 0;

        // Lifter positions
        int lifterMin =     0;
        int lifterMid =   971;
        int lifterMax = 11935;
        boolean lifterMoving = false;

        int intaking = 0;
        double spinSpeed = .7;

        boolean slower = false;
        double slowerSpeed = .5;

        // Toast notification because I can
        AppUtil.getInstance().showToast(UILocation.BOTH, "Ver 2 Iteration 0.1-0.0");
        waitForStart(); // This function halts the program until you press start

        // Pre-run
        // Copies of gamepads to save their state from the previous cycle (you'll see the use of that later)
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        while (opModeIsActive()) {
            // TeleOp loop
            drive.gyro();
            drive.Drive(
                    !slower ? -gamepad1.left_stick_x : -gamepad1.left_stick_x * slowerSpeed,
                    !slower ? gamepad1.left_stick_y : gamepad1.left_stick_y * slowerSpeed,
                    !slower ? -gamepad1.right_stick_x : -gamepad1.right_stick_x * slowerSpeed
            );

            if (gamepad1.back) { drive.resetAngle(); }

            lifterPower = -gamepad2.left_stick_y;
            int curPos = robot.getLifter().getCurrentPosition();
            if (lifterMoving) {
                if (lifterPower > 0.1 || lifterPower < -0.1) { lifterMoving = false; }
                if (curPos > 6653) { lifterPower = -1; }
                else if (curPos > 6652) { lifterMoving = false; lifterPower = 0; }
                else if (curPos < 6651) { lifterPower = 1; }
            } else if (curPos < lifterMid + 2) {
            } else if (curPos > lifterMax - 2) {
                lifterPower = clamp(-1, 0, lifterPower);
            } else if (curPos < lifterMin + 2) {
                lifterPower = clamp(0, 1, lifterPower);
            }
            
            if (gamepad2.y && !cp2.y) {
                lifterMoving = !lifterMoving;
            }

            if (intaking == 0 && gamepad2.a && !cp2.a) {
                lSrvPower = spinSpeed;
                rSrvPower = -spinSpeed;
                intaking = 1;
            } else if (intaking != 0 && ((gamepad2.a && !cp2.a) || (gamepad2.b && !cp2.b))) {
                lSrvPower = 0;
                rSrvPower = 0;
                intaking = 0;
            } else if (intaking == 0 && gamepad2.b && !cp2.b) {
                lSrvPower = -spinSpeed + .1;
                rSrvPower = spinSpeed - .1;
                intaking = -1;
            }

            if (intaking == 1 && robot.getTouch().isPressed()) {
                lSrvPower = 0;
                rSrvPower = 0;
                intaking = 0;
            }

            if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                rSrvPower = 1;
                lSrvPower = 0;
                intaking = 0;
            } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                lSrvPower = 0;
                rSrvPower = -1;
                intaking = 0;
            } else if (intaking == 0) {
                lSrvPower = 0;
                rSrvPower = 0;
            }

            armPos += (int) gamepad2.right_stick_x * 100;


            if (gamepad1.x && !cp1.x) { slower = !slower; }

            if (lifterPower != lifterPrevP) { robot.getLifter().setPower(lifterPower); lifterPrevP = lifterPower; }
            if (armPos != armPrevP) { robot.getArm().setTargetPosition(armPos); armPrevP = armPos; }
            if (lSrvPower != lSrvPrevP) { robot.getLSrv().setPower(lSrvPower); lSrvPrevP = lSrvPower; }
            if (rSrvPower != rSrvPrevP) { robot.getRSrv().setPower(rSrvPower); rSrvPrevP = rSrvPower; }
        }
    }

    // Clamp function because there is no native clamp
    private double clamp(double min, double max, double value) {
        return Math.min(max, Math.max(value, min));
    }
}
