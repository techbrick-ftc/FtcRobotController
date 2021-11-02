package org.firstinspires.ftc.teamcode;
// 10700
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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
            if (degree) {
                ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (rx == 360){
                    if (ar.getCurrentPosition() >= 180 * 10528 / 360) {
                        ar.setTargetPosition((int)(360 * 10528 / 360));
                    }
                    else if (ar.getCurrentPosition() < 180 * 10528 / 360) {
                        ar.setTargetPosition(0);
                    }
                }
                else {
                    ar.setTargetPosition((int)(rx * 10528 / 360));
                }
                ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ar.setPower(1);
            }
            else {
                ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ar.setTargetPosition((int)(Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) * 10528 / (2 * Math.PI)));
                ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ar.setPower(Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y));
                telemetry.addLine("" + Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) * 10528 / (2 * Math.PI));
                telemetry.update();
            }
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
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Loading Black Matter...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Black Matter Generated...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        double turningspeed = 1;
        Gamepad cp1 = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        while (opModeIsActive()) {
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            drive(x2, y2, rx);
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
            if (gamepad2.dpad_up) {
                arm(360, true);
            }
            if (gamepad2.dpad_right) {
                arm(90, true);
            }
            if (gamepad2.dpad_down) {
                arm(180, true);
            }
            if (gamepad2.dpad_left) {
                arm(270, true);
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
            } catch (Exception ignored) {}

        }
    }
}