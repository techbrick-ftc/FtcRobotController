package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous(name="Victory Dance")
public class VictoryDance extends LinearOpMode {
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ar;
    DcMotorEx ap;
    public void runOpMode() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        fl.setPower(-800);
        fr.setPower(800);
        rl.setPower(-800);
        rr.setPower(800);
        sleep(7500);
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
}