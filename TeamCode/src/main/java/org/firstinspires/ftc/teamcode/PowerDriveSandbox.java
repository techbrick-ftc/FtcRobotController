package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="PowerDrive - Sandbox Drive")
public class PowerDriveSandbox extends LinearOpMode {
    DcMotorEx fl; DcMotorEx fr; DcMotorEx rl; DcMotorEx rr;
    DcMotorEx[] all;
    boolean fieldCentric = true;
    float straightX;
    float straightY;
    boolean straight = false;
    boolean running = false;
    boolean InRange(double starting, double checked, double range) {
        return Math.abs(Math.abs(starting) - Math.abs(checked)) < range;
    }
    Runnable StraightCheck(float x, float y) {
        straightX = x;
        straightY = y;
        running = true;
        ElapsedTime timer = new ElapsedTime();
        while (running) {
            if (!InRange(x, gamepad1.left_stick_x, 0.15) || !InRange(y, gamepad1.left_stick_y, 0.15)) {
                running = false;
            }
            else if (timer.milliseconds() > 600) {
                break;
            }
        }
        straight = running;
        running = false;
        return null;
    }
    public int intType(float input) {
        if (input < 0) {
            return -1;
        }
        else if (input > 0) {
            return 1;
        }
        else {
            return 0;
        }
    }
    public void powerMove(float x, float y) {
        if (!fieldCentric) {
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            if (x > 0.1 || x < -0.1) {
                double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
                 frontLeftPower = (y + x) / denominator;
                backLeftPower = (y - x) / denominator;
                frontRightPower = (y - x) / denominator;
                backRightPower = (y + x) / denominator;
            }
            else {
                frontLeftPower = Math.abs(y);
                backLeftPower = Math.abs(y);
                frontRightPower = Math.abs(y);
                backRightPower = Math.abs(y);
            }

            fl.setVelocity(frontLeftPower * 1300);
            rl.setVelocity(backLeftPower * 1300);
            fr.setVelocity(frontRightPower * 1300);
            rr.setVelocity(backRightPower * 1300);
        }
    }
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        all = new DcMotorEx[]{fl, fr, rl, rr};
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Powering Up...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor : all) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        Gamepad cp1 = new Gamepad();
        waitForStart();
        telemetry.addLine("Powered on...");
        while (opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.05 || Math.abs(gamepad1.left_stick_y) > 0.05) {
                if (!(InRange(straightX, gamepad1.left_stick_x, 0.125) && InRange(straightY, gamepad1.left_stick_y, 0.125))) {
                    straight = false;
                }
                if (straight) {
                    powerMove(straightX, straightY);
                }
                else {
                    powerMove(gamepad1.left_stick_x, gamepad1.left_stick_y);
                }
                if (!running && !straight) {
                    Thread th = new Thread(StraightCheck(gamepad1.left_stick_x, gamepad1.left_stick_y));
                    th.start();
                }
            }
            idle();
            try {
                cp1 = gamepad1;
            } catch (Exception ignored) {}
        }
        for (DcMotorEx motor : all) {
            motor.setPower(0);
        }
    }
}