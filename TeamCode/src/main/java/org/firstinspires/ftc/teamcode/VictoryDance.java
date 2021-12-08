package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.Globals;

@Disabled
@Autonomous(name="VictoryDance")
public class VictoryDance extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    DcMotor ar;
    DcMotor ap;
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        ar = hardwareMap.get(DcMotor.class, "ar");
        ap = hardwareMap.get(DcMotor.class, "ap");
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Loading...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Victory Dance has begun...");
        telemetry.update();
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();
        while (opModeIsActive()) {
            timer1.reset();
            timer2.reset();
            while (timer1.seconds() < 4) {
                fl.setPower(0.8);
                fr.setPower(0.8);
                rl.setPower(0.8);
                rr.setPower(0.8);
                ar.setPower(0.8);
                while (timer2.seconds() < 2) {
                    ap.setPower(-0.7);
                }
                ap.setPower(0);
                timer2.reset();
                while (timer2.seconds() < 2) {
                    ap.setPower(0.7);
                }
            }
            fl.setPower(0);
            fr.setPower(0);
            rl.setPower(0);
            rr.setPower(0);
            ar.setPower(0);
            ap.setPower(0);
            timer1.reset();
            timer2.reset();
            while (timer1.seconds() < 4) {
                fl.setPower(-0.8);
                fr.setPower(-0.8);
                rl.setPower(-0.8);
                rr.setPower(-0.8);
                ar.setPower(-0.8);
                while (timer2.seconds() < 2) {
                    ap.setPower(-0.7);
                }
                ap.setPower(0);
                timer2.reset();
                while (timer2.seconds() < 2) {
                    ap.setPower(0.7);
                }
            }
            fl.setPower(0);
            fr.setPower(0);
            rl.setPower(0);
            rr.setPower(0);
        }
    }
}