package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name=" ")
public class hecks extends LinearOpMode {
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ap;
    public void runOpMode() {
        //Setting variables
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        ap.setDirection(DcMotorSimple.Direction.REVERSE);
        //Waits for start of OpMode
        waitForStart();
        fl.setVelocity(-900);
        fr.setVelocity(900);
        rl.setVelocity(-900);
        rr.setVelocity(900);
        ap.setVelocity(500);
        sleep(2500);
        fl.setVelocity(0);
        fr.setVelocity(0);
        rl.setVelocity(0);
        rr.setVelocity(0);
        ap.setVelocity(0);
    }
}