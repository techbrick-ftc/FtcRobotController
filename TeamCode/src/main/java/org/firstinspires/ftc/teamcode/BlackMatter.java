package org.firstinspires.ftc.teamcode;
// 10700
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Black Matter")
public class BlackMatter extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    DcMotor ar;
    DcMotor ap;
    CRServo cs1;
    CRServo cs2;
    TouchSensor ts1;
    boolean autorunning = false;
    double angle = 0;
    double driveSpeed = 1.00;
    int motor = 1;
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
    public void arm(double rx, boolean degree) {
        if (motor == 1) {
            ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ar.setTargetPosition((int)(Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) * 10528 / (2 * Math.PI)));
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setPower(Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y));
            telemetry.addLine("" + Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) * 10528 / (2 * Math.PI));
            telemetry.update();
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
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Loading Black Matter...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Black Matter Generated...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        double turningspeed = 1;
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        while (opModeIsActive()) {
            if (ts1.isPressed() && gamepad2.left_trigger < 0.5) {
                cs1.setPower(0);
                cs2.setPower(0);
            }
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            drive(x2, y2, rx);
            while (gamepad2.left_trigger > 0.5 && opModeIsActive()) {
                cs1.setPower(-0.50);
                cs2.setPower(0.28);
            }
            if (gamepad2.left_bumper) {
                while (!ts1.isPressed() && opModeIsActive() && gamepad2.left_trigger < 0.5) {
                    cs1.setPower(0.50);
                    cs2.setPower(-0.28);
                    while (gamepad2.dpad_up) {
                        ap.setPower(-0.4);
                    }
                    while (gamepad2.dpad_down) {
                        ap.setPower(0.2);
                    }
                    while (gamepad2.dpad_left) {
                        ar.setPower(-0.4);
                    }
                    while (gamepad2.dpad_right) {
                        ar.setPower(0.4);
                    }
                    ar.setPower(0);
                    ap.setPower(0);
                    Globals.getImu().getPosition();
                    drive(gamepad1.left_stick_x * 1.1, -gamepad1.left_stick_y, gamepad1.right_stick_x * turningspeed);
                }
            }
            while (gamepad2.dpad_up) {
                ap.setPower(-0.4);
            }
            while (gamepad2.dpad_down) {
                ap.setPower(0.2);
            }
            if (gamepad2.a) {
                motor = 1;
            }
            if (gamepad2.b) {
                motor = 2;
            }
            if (gamepad2.x) {
                motor = 3;
            }
            if (gamepad2.y) {
                motor = 4;
            }
            if (gamepad2.left_stick_x > 0.1 || gamepad1.right_stick_y > 0.1) {
                arm(0, false);
            }
            while (gamepad2.dpad_left) {
                ar.setPower(-0.4);
            }
            while (gamepad2.dpad_right) {
                ar.setPower(0.4);
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
                timer2.reset();
                while (gamepad1.right_trigger > 0.5) {
                    if (timer2.seconds() > 1) {
                        driveSpeed = 0.25;
                    }
                }
            }
            if (gamepad1.right_trigger > 0.5 && cp1.right_trigger <= 0.5) {
                driveSpeed += 0.25;
                if (driveSpeed == 1.25) {
                    driveSpeed = 1.00;
                }
                timer2.reset();
                while (gamepad1.right_trigger > 0.5) {
                    if (timer2.seconds() > 1) {
                        driveSpeed = 1;
                    }
                }
            }
            if (gamepad1.a && !cp1.a) {
                turningspeed += 0.25;
                if (turningspeed == 1.25) {
                    turningspeed = 0.25;
                }
                timer1.reset();
                while (gamepad1.a) {
                    if (timer1.seconds() > 1) {
                        turningspeed = 1;
                    }
                }
            }
            try {
                cp1.copy(gamepad1);
                cs1.setPower(0);
                cs2.setPower(0);
                ar.setPower(0);
                ap.setPower(0);
            } catch (Exception ignored) {}
            try {
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
        }
    }
}