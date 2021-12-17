package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Minecraft Beta")
public class MinecraftBeta extends LinearOpMode {
    //Global Variables
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ar;
    DcMotorEx ap;
    //
    //RobotCentric Drive Control
    //
    public void robotCentricDrive(double y2) {
        if (y2 > 0.05) {
            fl.setTargetPosition(fl.getCurrentPosition() + 43);
            fr.setTargetPosition(fl.getCurrentPosition() + 43);
            rl.setTargetPosition(fl.getCurrentPosition() + 43);
            rr.setTargetPosition(fl.getCurrentPosition() + 43);
            fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fl.setVelocity(y2 * 2000);
            rl.setVelocity(y2 * 2000);
            fr.setVelocity(y2 * 2000);
            rr.setVelocity(y2 * 2000);
        }
        if (y2 < -0.05) {
            fl.setTargetPosition(fl.getCurrentPosition() - 43);
            fr.setTargetPosition(fl.getCurrentPosition() - 43);
            rl.setTargetPosition(fl.getCurrentPosition() - 43);
            rr.setTargetPosition(fl.getCurrentPosition() - 43);
            fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fl.setVelocity(y2 * 2000);
            rl.setVelocity(y2 * 2000);
            fr.setVelocity(y2 * 2000);
            rr.setVelocity(y2 * 2000);
        }
    }
    //
    //Turn Control
    //
    public void turn(double turn) {
        if (turn > 0.05) {
            fl.setTargetPosition(-(2850 * 20 / 360));
            fr.setTargetPosition(2850 * 20 / 360);
            rl.setTargetPosition(-(2850 * 20 / 360));
            rr.setTargetPosition(2850 * 20 / 360);
            fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fl.setVelocity(-1200);
            fr.setVelocity(1200);
            rl.setVelocity(-1200);
            rr.setVelocity(1200);
        }
        if (turn < -0.05)
        {
            fl.setTargetPosition(2850 * 20 / 360);
            fr.setTargetPosition(-(2850 * 20 / 360));
            rl.setTargetPosition(2850 * 20 / 360);
            rr.setTargetPosition(-(2850 * 20 / 360));
            fl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            rr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            fl.setVelocity(1200);
            fr.setVelocity(-1200);
            rl.setVelocity(1200);
            rr.setVelocity(-1200);
        }
    }
    //
    //Arm Rotational Control
    //
    public void armRotation() {
        double onx = gamepad2.left_stick_x;
        if (onx > 0.05 && (gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(onx * 800);
        }
        else if (onx < -0.05 && (gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(-onx * 800);
        }
        else {
            ar.setVelocity(0);
        }
    }
    //
    //Arm Pitch Control
    //
    public void armPitch() {
        //Arm up
        if ((gamepad2.left_bumper || gamepad2.dpad_up) && (ap.getCurrentPosition() > -5650 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(-1400);
        }
        //Arm down
        else if ((gamepad2.left_trigger > 0.05 || gamepad2.dpad_down) && ( ap.getCurrentPosition() < 0 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(1400);
        }
        //Checks if not busy
        else if (!ap.isBusy()) {
            ap.setVelocity(0);
        }
    }
    @Override
    //OpMode
    public void runOpMode() {
        //Setting variables
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        ar.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ap.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Loading Minecraft Beta... 0%. Press start to begin");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Loading Minecraft Beta... 20%");
        sleep(1000);
        telemetry.addLine("Loading Minecraft Beta... 40%");
        sleep(1000);
        telemetry.addLine("Loading Minecraft Beta... 60%");
        sleep(1000);
        telemetry.addLine("Loading Minecraft Beta... 80%");
        sleep(1000);
        telemetry.addLine("Minecraft Loaded... 100%");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive()) {
            if (!fl.isBusy()) {
                double y2 = -gamepad1.left_stick_y;
                robotCentricDrive(y2);
            }
            if (!fl.isBusy()) {
                double rx = gamepad1.right_stick_x;
                turn(rx);
            }
            armPitch();
            armRotation();
            sleep(70);
        }
    }
}