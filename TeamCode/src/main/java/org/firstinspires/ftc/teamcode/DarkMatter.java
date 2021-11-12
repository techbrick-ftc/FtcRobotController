package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Dark Matter (◉_◉)")
public class DarkMatter extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    DcMotor ar;
    DcMotor ap;
    CRServo cs1;
    CRServo cs2;
    TouchSensor ts1;
    TouchSensor ts2;
    boolean autorunning = false;
    double angle = 0;
    double apEncoderTicks = 0;
    double driveSpeed = 1.00;
    boolean runningInput = false;
    public void drive(double x2, double y2, double rx) {
        double x = x2;
        double y = y2;
        if (!autorunning) {
            Orientation orientation = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            x = x2 * Math.cos(-(orientation.firstAngle - angle)) - y2 * Math.sin(-(orientation.firstAngle - angle));
            y = y2 * Math.cos(-(orientation.firstAngle - angle)) - x2 * Math.sin(-(orientation.firstAngle - angle));
        }
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setPower(frontLeftPower * driveSpeed);
        rl.setPower(backLeftPower * driveSpeed);
        fr.setPower(frontRightPower * driveSpeed);
        rr.setPower(backRightPower * driveSpeed);

    }
    public void arm() {
        if (gamepad2.left_stick_x > 0.1) {
            ar.setPower(Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y));
        }
        if (gamepad2.left_stick_x < 0.1) {
            ar.setPower(-Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y));
        }
    }
    public void autorun(double x, double y) {
        while (autorunning && opModeIsActive()) {
            drive(x, y, gamepad1.right_stick_x);
            if (gamepad1.back) {
                autorunning = false;
            }
            if (gamepad1.left_trigger > 0.5) {
                driveSpeed -= 0.25;
                if (driveSpeed == 0) {
                    driveSpeed = 0.25;
                }
            }
            if (gamepad1.right_trigger > 0.5) {
                driveSpeed += 0.25;
                if (driveSpeed == 1.25) {
                    driveSpeed = 1.00;
                }
            }
        }
    }
    @Override

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        ar = hardwareMap.get(DcMotor.class, "ar");
        ap = hardwareMap.get(DcMotor.class, "ap");
        cs1 = hardwareMap.get(CRServo.class, "cs1");
        cs2 = hardwareMap.get(CRServo.class, "cs2");
        ts1 = hardwareMap.get(TouchSensor.class, "ts1");
        ts2 = hardwareMap.get(TouchSensor.class, "ts2");
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Injecting Dark Matter...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Injecting Dark Matter...");
        telemetry.addLine("The world is consumed...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        double turningspeed = 1;
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        while (opModeIsActive()) {
            telemetry.addData("CURRENT.POS", ap.getCurrentPosition());
            telemetry.update();
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            drive(x2, y2, rx);
            if (gamepad2.dpad_up) {
                ap.setPower(-0.75);
            }
            else if (gamepad2.dpad_down) {
                ap.setPower(0.65);
            }
            else {
                ap.setPower(0);
            }
            if (gamepad2.a) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(0);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Math.abs(ap.getCurrentPosition()) > 20) {
                    ap.setPower(0.66);
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            else if (gamepad2.y) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(-3050);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Math.abs(ap.getCurrentPosition()) < 3060 || Math.abs(ap.getCurrentPosition()) > 3040) {
                    ap.setPower(-0.66);
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            else if (gamepad2.x) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(-2116);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Math.abs(ap.getCurrentPosition()) < 2126 || Math.abs(ap.getCurrentPosition()) > 2106) {
                    ap.setPower(-0.66);
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            else {
                ap.setPower(0);
            }
            if (ts2.isPressed()) {
                ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad2.right_trigger > 0.5) {
                cs1.setPower(-0.50);
                cs2.setPower(0.35);
            }
            else if (!ts1.isPressed() && gamepad2.right_bumper) {
                cs1.setPower(0.50);
                cs2.setPower(-0.35);
                runningInput = true;
            }
            else if (ts1.isPressed() || gamepad2.right_trigger > 0.2) {
                cs1.setPower(-0.50);
                cs2.setPower(0.35);
                runningInput = false;
                sleep(4);
                cs1.setPower(0);
                cs2.setPower(0);
            }
            else {
                cs1.setPower(0);
                cs2.setPower(0);
            }

            if (gamepad2.left_stick_x > 0.1 || gamepad1.right_stick_y > 0.1) {
                arm();
            }
            if (gamepad2.dpad_left) {
                ar.setPower(-0.4);
            }
            else if (gamepad2.dpad_right) {
                ar.setPower(0.4);
            }
            else {
                ar.setPower(0);
            }
            if (gamepad1.left_stick_button) {
                angle = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            }
            if (gamepad1.dpad_up) {
                autorunning = true;
                autorun(0, 1);
            }
            if (gamepad1.dpad_down) {
                autorunning = true;
                autorun(0, -1);
            }
            if (gamepad1.dpad_left) {
                autorunning = true;
                autorun(-1, 0);
            }
            if (gamepad1.dpad_right) {
                autorunning = true;
                autorun(1, 0);
            }
            if (gamepad1.left_trigger > 0.5 && cp1.left_trigger <= 0.5) {
                driveSpeed -= 0.25;
                if (driveSpeed == 0) {
                    driveSpeed = 0.25;
                }
            }
            else if (gamepad1.right_trigger > 0.5 && cp1.right_trigger <= 0.5) {
                driveSpeed += 0.25;
                if (driveSpeed == 1.25) {
                    driveSpeed = 1.00;
                }
            }
            if (gamepad1.a && !cp1.a) {
                turningspeed += 0.25;
                if (turningspeed == 1.25) {
                    turningspeed = 0.25;
                }
            }
            try {
                cp1.copy(gamepad1);
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
            if (runningInput) {
                cs1.setPower(0.50);
                cs2.setPower(-0.35);
            }
            arm();
        }
    }
}