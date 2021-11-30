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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;

@TeleOp(name="Main",group="")
public class Main extends LinearOpMode {
    // Pre-init
    private final Nikolaj robot = new Nikolaj();
    private final FieldCentric drive = new FieldCentric();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();
    
    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);

        final CRServo intake = robot.getIntake();
        final CRServo spinner = robot.getSpinner();

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu1");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationDataFile = "BNO055IMUCalibration.json";
        params.loggingEnabled = false;
        params.loggingTag = "LolNoThanks";
        imu.initialize(params);

        drive.setUp(
            new DcMotor[]{robot.frMotor(), robot.rrMotor(), robot.rlMotor(), robot.flMotor()},
            new double[]{PI/4, 3*PI/4, 5*PI/4, 7*PI/4},
            imu,
            hardwareMap
        );

        int lifterMin = -8000;
        int lifterMax = 0;
        double lifterSpeed;

        int intaking = 0;
        double spinSpeed = .7;

        boolean slower = false;
        double slowerSpeed = .5;

        int loops = 1;

        AppUtil.getInstance().showToast(UILocation.BOTH, "Iteration 7.3-8.3");
        waitForStart();

        // Pre-run
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        while (opModeIsActive()) {
            // TeleOp loop
            drive.gyro();
            drive.Drive(
                    !slower ? -gamepad1.left_stick_x : -gamepad1.left_stick_x * slowerSpeed,
                    !slower ? gamepad1.left_stick_y : gamepad1.left_stick_y * slowerSpeed,
                    !slower ? -gamepad1.right_stick_x * .5 : -gamepad1.right_stick_x * slowerSpeed * .5
            );
            
            if (gamepad1.back && !cp1.back) { drive.resetAngle(); }

            lifterSpeed = gamepad2.left_stick_y;
            int curPos = robot.getLifter().getCurrentPosition();
            if (curPos > lifterMax - 2) {
                lifterSpeed = clamp(-1, 0, lifterSpeed);
            } else if (curPos < lifterMin + 2) {
                lifterSpeed = clamp(0, 1, lifterSpeed);
            }
            robot.getLifter().setPower(lifterSpeed);

            if (intaking == 0 && gamepad2.a && !cp2.a) {
                intake.setPower(spinSpeed);
                spinner.setPower(-spinSpeed);
                intaking = 1;
            } else if (intaking != 0 && ((gamepad2.a && !cp2.a) || (gamepad2.b && !cp2.b))) {
                intake.setPower(0);
                spinner.setPower(0);
                intaking = 0;
            } else if (intaking == 0 && gamepad2.b && !cp2.b) {
                intake.setPower(-spinSpeed);
                spinner.setPower(spinSpeed);
                intaking = -1;
            }

            if (intaking == 1 && robot.getTouch().isPressed()) {
                robot.getIntake().setPower(0);
                robot.getSpinner().setPower(0);
                intaking = 0;
            }

            if (gamepad2.right_bumper && !gamepad2.left_bumper) {
                spinner.setPower(spinSpeed);
                intake.setPower(0);
                intaking = 0;
            } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
                spinner.setPower(-spinSpeed);
                intake.setPower(0);
                intaking = 0;
            } else if (intaking == 0) {
                spinner.setPower(0);
            }

            if (gamepad1.x && !cp1.x) {
                slower = !slower;
            }
            telemetry.addData("Slower", slower);

            telemetry.update();

            try {
                cp1.copy(gamepad1);
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
        }
    }

    private double clamp(double min, double max, double value) {
        return Math.min(max, Math.max(value, min));
    }
}
