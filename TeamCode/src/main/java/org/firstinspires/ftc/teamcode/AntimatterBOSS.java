package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Antimatter BOSS")
public class AntimatterBOSS extends LinearOpMode {
    boolean Diagnostics = false;
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    boolean autorunning = false;
    double angle = 0;
    double driveSpeed = 1.00;
    private void Diagnostics_Show(Telemetry telemetry) {
        telemetry.addLine("gamepad1.left_stick_x:" + gamepad1.left_stick_x);
        telemetry.addLine("gamepad1.left_stick_y:" + gamepad1.left_stick_y);
        telemetry.addLine("gamepad1.right_stick_x:" + gamepad1.right_stick_x);
        telemetry.addLine("gamepad1.right_stick_y:" + gamepad1.right_stick_y);
        telemetry.addLine("gamepad1.a:" + gamepad1.a);
        telemetry.addLine("gamepad1.b:" + gamepad1.b);
        telemetry.addLine("gamepad1.x:" + gamepad1.x);
        telemetry.addLine("gamepad1.y:" + gamepad1.y);
        telemetry.addLine("gamepad1.dpad_left:" + gamepad1.dpad_left);
        telemetry.addLine("gamepad1.dpad_right:" + gamepad1.dpad_right);
        telemetry.addLine("gamepad1.dpad_up:" + gamepad1.dpad_up);
        telemetry.addLine("gamepad1.dpad_down:" + gamepad1.dpad_down);
        telemetry.addLine("gamepad1.left_trigger:" + gamepad1.left_trigger);
        telemetry.addLine("gamepad1.right_trigger:" + gamepad1.right_trigger);
        telemetry.addLine("gamepad1.start:" + gamepad1.start);
        telemetry.addLine("gamepad1.back:" + gamepad1.back);
        telemetry.addLine("fl.power:" + fl.getPower());
        telemetry.addLine("fr.power:" + fr.getPower());
        telemetry.addLine("rl.power:" + rl.getPower());
        telemetry.addLine("rr.power:" + rr.getPower());
        telemetry.addLine("IMU.firstAngle:" + Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addLine("IMU.secondAngle:" + Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).secondAngle);
        telemetry.addLine("IMU.thirdAngle:" + Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).thirdAngle);
        telemetry.addLine("IMU_SUBTRACTION_ANGLE:" + angle);
        telemetry.update();
    }
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
        if (Diagnostics) {
            Diagnostics_Show(telemetry);
        }

    }
    public void autorun(double x, double y) {
        while (autorunning && opModeIsActive()) {
            if (Diagnostics) {
                Diagnostics_Show(telemetry);
            }
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
        boolean victoryDance = false;
        boolean victoryDanceL = true;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        while (opModeIsActive()) {
            if (Diagnostics) {
                Diagnostics_Show(telemetry);
            }
            while (victoryDance && opModeIsActive() && timer.seconds() < 10) {
                if (Diagnostics) {
                    Diagnostics_Show(telemetry);
                }
                if (timer2.seconds() >= 2.5) {
                    timer2.reset();
                    victoryDanceL = !victoryDanceL;
                    timer2.startTime();
                }
                drive(0, 0, victoryDanceL ? -0.75 : 0.75);
            }
            if (timer.seconds() > 10) {
                victoryDance = false;
            }
            if (gamepad1.b) {
                victoryDance = true;
                timer.reset();
                timer.startTime();

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
                if (Diagnostics) {
                    Diagnostics_Show(telemetry);
                }
            }
            if (gamepad1.y && !cp1.y) {
                Diagnostics = !Diagnostics;
                telemetry.addLine("Diagnostics: " + Diagnostics);
                telemetry.update();
            }
            try {
                cp1.copy(gamepad1);
            } catch (Exception ignored) {}

        }
    }
}

