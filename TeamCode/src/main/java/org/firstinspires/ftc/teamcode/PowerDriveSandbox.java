package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Disabled
@TeleOp(name="PowerDrive - Sandbox Drive")
public class PowerDriveSandbox extends LinearOpMode {
    DcMotorEx fl; DcMotorEx fr; DcMotorEx rl; DcMotorEx rr;
    DcMotorEx ar; DcMotorEx ap;
    CRServo cs1; CRServo cs2;
    TouchSensor ts1; TouchSensor ts2;
    DcMotorEx led;
    DcMotorEx[] all;
    boolean fieldCentric = true;
    boolean straightOnly = false;
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
            if (straightOnly) {
                fl.setVelocity(intType(y) * 1800);
                rl.setVelocity(intType(y) * 1800);
                fr.setVelocity(intType(y) * 1800);
                rr.setVelocity(intType(y) * 1800);
            }
            else {
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

                fl.setVelocity(frontLeftPower * 1800);
                rl.setVelocity(backLeftPower * 1800);
                fr.setVelocity(frontRightPower * 1800);
                rr.setVelocity(backRightPower * 1800);
            }
        }
    }
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        led = hardwareMap.get(DcMotorEx.class, "B1");
        cs1 = hardwareMap.get(CRServo.class, "cs1");
        cs2 = hardwareMap.get(CRServo.class, "cs2");
        ts1 = hardwareMap.get(TouchSensor.class, "ts1");
        ts2 = hardwareMap.get(TouchSensor.class, "ts2");
        all = new DcMotorEx[]{fl, fr, rl, rr, ap, ar, led};
        ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Powering Up...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor : all) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        waitForStart();

        while (opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_x) > 0.05 || Math.abs(gamepad1.left_stick_y) > 0.05) {
                powerMove(gamepad1.left_stick_x, gamepad1.left_stick_y);
            }
            idle();
            try {
                cp1 = gamepad1;
                cp2 = gamepad2;
            } catch (Exception ignored) {}
        }
        for (DcMotorEx motor : all) {
            motor.setPower(0);
        }
    }
}