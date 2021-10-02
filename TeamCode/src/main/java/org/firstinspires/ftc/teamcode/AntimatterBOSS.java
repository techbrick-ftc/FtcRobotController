package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Antimatter BOSS")
public class AntimatterBOSS extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    boolean autorunning = false;
    double angle = 0;
    double driveSpeed = 1;
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
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Loading...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("The War has Begun...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        double turningspeed = 1;
        Gamepad cp1 = new Gamepad();
        int victoryDance = 0;
        boolean victoryDanceL = true;

        while (opModeIsActive()) {
            while (victoryDance > 0 && opModeIsActive()) {
                if (victoryDance % 100 == 0) {
                    victoryDanceL = !victoryDanceL;
                }
                drive(0, 0, victoryDanceL ? -0.75 : 0.75);

                victoryDance -= 1;
            }
            if (gamepad1.b) {
                victoryDance = 500;
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
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            drive(x2, y2, rx);
            if (gamepad1.a) {
                turningspeed += 0.25;
                if (turningspeed == 1.25) {
                    turningspeed = 0.25;
                }
            }
            if (gamepad1.left_stick_button) {
                angle = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            }
            try {
                cp1.copy(gamepad1);
            } catch (Exception ignored) {}

        }
    }
}

