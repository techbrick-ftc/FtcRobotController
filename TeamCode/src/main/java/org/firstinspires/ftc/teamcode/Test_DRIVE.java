package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TESTDRIVE.EXE")
public class Test_DRIVE extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx rl = hardwareMap.get(DcMotorEx.class, "rl");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        fl.setTargetPosition(fl.getTargetPosition() + 10);
        fr.setTargetPosition(fr.getTargetPosition() - 10);
        rl.setTargetPosition(rl.getTargetPosition() + 10);
        rr.setTargetPosition(rr.getTargetPosition() - 10);
        fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        fl.setPower(1);
        fr.setPower(1);
        rl.setPower(1);
        rr.setPower(1);
        while (opModeIsActive()) {
        }
    }
}


