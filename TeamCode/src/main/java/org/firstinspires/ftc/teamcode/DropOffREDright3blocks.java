package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="RED DROP OFF: RIGHT 3 blocks")
public class DropOffREDright3blocks extends LinearOpMode {
    //Global Variables
    //Ticks For Yaw: 2850 * angle / 360
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ar;
    DcMotorEx ap;
    DcMotor led;
    CRServo cs1;
    CRServo cs2;
    TouchSensor ts1;
    TouchSensor tsleft;
    TouchSensor tsright;
    final double tpi_s = 46.5567;
    final double tpi_d = 43.0301;
    final int ticksHighPitch = -1585;
    final int ticksMiddlePitch = -818;
    final int ticksLowPitch = -409;
    final int ticksDegree90Yaw = -712;
    final int ticksDegree270Yaw = -2138;
    final int ticksDegree180Yaw = -1425;
    public void runInches(int inches, direction direct, double speed) {
        if (direct == direction.forward) {
            fl.setTargetPosition((int)(Math.round(tpi_d * inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_d * inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_d * inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_d * inches) + rr.getCurrentPosition()));
        }
        else if (direct == direction.backward) {
            fl.setTargetPosition((int)(Math.round(tpi_d * -inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_d * -inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_d * -inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_d * -inches) + rr.getCurrentPosition()));
        }
        else if (direct == direction.right) {
            fl.setTargetPosition((int)(Math.round(tpi_s * inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_s * -inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_s * -inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_s * inches) + rr.getCurrentPosition()));
        }
        else if (direct == direction.left) {
            fl.setTargetPosition((int)(Math.round(tpi_s * -inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_s * inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_s * inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_s * -inches) + rr.getCurrentPosition()));
        }
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setVelocity(speed);
        fr.setVelocity(speed);
        rl.setVelocity(speed);
        rr.setVelocity(speed);
    }
    public void armPos(armPositionsPitch positionPitch, int speedPitch, armPositionsYaw positionYaw, int speedYaw) {
        if (positionPitch == armPositionsPitch.intake) {
            ap.setTargetPosition(0);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionPitch == armPositionsPitch.middle) {
            ap.setTargetPosition(ticksMiddlePitch);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionPitch == armPositionsPitch.output) {
            ap.setTargetPosition(ticksHighPitch);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionPitch == armPositionsPitch.low) {
            ap.setTargetPosition(ticksLowPitch);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionPitch == armPositionsPitch.up) {
            ap.setTargetPosition(-2557);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionPitch == armPositionsPitch.lvl1) {
            ap.setTargetPosition(-332);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (positionYaw == armPositionsYaw.start) {
            ar.setTargetPosition(0);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionYaw == armPositionsYaw.degree90) {
            ar.setTargetPosition(ticksDegree90Yaw);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionYaw == armPositionsYaw.degree180) {
            ar.setTargetPosition(ticksDegree180Yaw);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionYaw == armPositionsYaw.degree270) {
            ar.setTargetPosition(ticksDegree270Yaw);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionYaw == armPositionsYaw.degree225) {
            ar.setTargetPosition(2850 * -225 / 360);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        ap.setVelocity(speedPitch);
        ar.setVelocity(speedYaw);
    }
    public void armPosDegree(int positionPitch, int speedPitch, int positionYaw, int speedYaw) {
        if (positionPitch != 0) {
            ap.setTargetPosition(-9228 * positionPitch / 360);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(speedPitch);
        }
        if (positionYaw != 0) {
            ar.setTargetPosition(2850 * positionYaw / 360);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setVelocity(speedYaw);
        }
    }
    public void toEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runOpMode() {
//Setting variables
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        led = hardwareMap.get(DcMotor.class, "B1");
        cs1 = hardwareMap.get(CRServo.class, "cs1");
        cs2 = hardwareMap.get(CRServo.class, "cs2");
        ts1 = hardwareMap.get(TouchSensor.class, "ts1");
        tsleft = hardwareMap.get(TouchSensor.class, "tsleft");
        tsright = hardwareMap.get(TouchSensor.class, "tsright");
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
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Injecting Dark Matter Automatically...");
        telemetry.update();
//Waits for start of OpMode
        waitForStart();
        telemetry.addLine("Injecting Dark Matter Automatically...");
        telemetry.addLine("The world is automatically being consumed...");
        telemetry.update();
        led.setPower(1);
        runInches(24, direction.left, 1100);
        sleep(100);
        armPos(armPositionsPitch.middle, 2500, armPositionsYaw.current, 0);
        sleep(200);
        armPosDegree(0, 0, -200, 1500);
        sleep(1400);
        toEncoder();
        if (tsleft.isPressed() && !tsright.isPressed()) {
            runInches(6, direction.right, 800);
            armPos(armPositionsPitch.lvl1, 2200, armPositionsYaw.current, 0);
            sleep(800);
            armPosDegree(0, 0, -236, 700);
            sleep(400);
        }
        else if (!tsleft.isPressed() && tsright.isPressed()) {
            armPos(armPositionsPitch.middle, 2200, armPositionsYaw.current, 0);
            sleep(800);
            armPosDegree(0, 0, -248, 900);
            sleep(400);
        }
        else {
            armPos(armPositionsPitch.output, 2500, armPositionsYaw.current, 0);
            sleep(800);
            armPosDegree(0, 0, -248, 900);
            sleep(400);
        }
        toEncoder();
        cs1.setPower(-0.45);
        cs2.setPower(0.45);
        sleep(1200);
        cs1.setPower(0);
        cs2.setPower(0);
//PICKUP #1
        if (tsleft.isPressed() && !tsright.isPressed()) {
            runInches(28, direction.right, 1200);
        }
        else {
            runInches(34, direction.right, 1200);
        }
        sleep(500);
        armPos(armPositionsPitch.low, 2200, armPositionsYaw.degree90, 1300);
        sleep(1700);
        toEncoder();
        runInches(21, direction.forward, 1100);
        armPos(armPositionsPitch.intake, 2000, armPositionsYaw.current, 0);
        while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
        toEncoder();
        cs1.setPower(0.55);
        cs2.setPower(-0.55);
        int prevCurrent = fl.getCurrentPosition();
        fl.setVelocity(500);
        fr.setVelocity(500);
        rl.setVelocity(500);
        rr.setVelocity(500);
        while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (23 * tpi_d) && opModeIsActive()) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        boolean on = false;
        if (fl.getCurrentPosition() < prevCurrent + (23 * tpi_d)) {
            fl.setVelocity(0);
            fr.setVelocity(0);
            rl.setVelocity(0);
            rr.setVelocity(0);
            cs1.setPower(0);
            cs2.setPower(0);
            on = true;
            armPos(armPositionsPitch.output, 2500, armPositionsYaw.degree225, 800);
            sleep(200);
            runInches((int) Math.round((fl.getCurrentPosition() - prevCurrent) / tpi_d + 27), direction.backward, 1300);
            sleep(2300);
            toEncoder();
            runInches(21, direction.left, 1300);
            sleep(1500);
            toEncoder();
            cs1.setPower(-0.55);
            cs2.setPower(0.55);
            sleep(900);
            cs1.setPower(0);
            cs2.setPower(0);
//Pickup #2
            runInches(25, direction.right, 1200);
            sleep(400);
            armPos(armPositionsPitch.low, 2200, armPositionsYaw.degree90, 1300);
            sleep(1100);
            toEncoder();
            runInches(27, direction.forward, 1100);
            armPos(armPositionsPitch.intake, 2200, armPositionsYaw.current, 0);
            while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
            cs1.setPower(1);
            cs2.setPower(-1);
            toEncoder();
            prevCurrent = fl.getCurrentPosition();
            fl.setVelocity(500);
            fr.setVelocity(500);
            rl.setVelocity(500);
            rr.setVelocity(500);

            while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (25 * tpi_d) && opModeIsActive()) {
                telemetry.addData("FL", fl.getCurrentPosition());
                telemetry.addData("FR", fr.getCurrentPosition());
                telemetry.addData("RL", rl.getCurrentPosition());
                telemetry.addData("RR", rr.getCurrentPosition());
                telemetry.addData("AR", ar.getCurrentPosition());
                telemetry.addData("AP", ap.getCurrentPosition());
                telemetry.update();
            }
        }
        if (fl.getCurrentPosition() < prevCurrent + (25 * tpi_d) && on) {
            fl.setVelocity(0);
            fr.setVelocity(0);
            rl.setVelocity(0);
            rr.setVelocity(0);
            cs1.setPower(0);
            cs2.setPower(0);
            armPos(armPositionsPitch.output, 2500, armPositionsYaw.degree225, 800);
            sleep(200);
            runInches((int)Math.round(((fl.getCurrentPosition()) - prevCurrent) / tpi_d + 27), direction.backward, 1300);
            sleep(2600);
            toEncoder();
            runInches(21, direction.left, 1300);
            sleep(1500);
            toEncoder();
            cs1.setPower(-0.55);
            cs2.setPower(0.55);
            sleep(900);
            cs1.setPower(0);
            cs2.setPower(0);
//Park
            runInches(25, direction.right, 1200);
            sleep(400);
            armPos(armPositionsPitch.low, 2200, armPositionsYaw.current, 0);
            armPosDegree(0, 0, -78, 1200);
            sleep(1100);
            toEncoder();
            runInches(27, direction.forward, 1400);
            armPos(armPositionsPitch.intake, 2200, armPositionsYaw.current, 0);
            while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
            toEncoder();
            cs1.setPower(1);
            cs2.setPower(-1);
            fl.setVelocity(500);
            fr.setVelocity(500);
            rl.setVelocity(500);
            rr.setVelocity(500);
            while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (24 * tpi_d) && opModeIsActive()) {
                telemetry.addData("FL", fl.getCurrentPosition());
                telemetry.addData("FR", fr.getCurrentPosition());
                telemetry.addData("RL", rl.getCurrentPosition());
                telemetry.addData("RR", rr.getCurrentPosition());
                telemetry.addData("AR", ar.getCurrentPosition());
                telemetry.addData("AP", ap.getCurrentPosition());
                telemetry.update();
            }
            cs1.setPower(0);
            cs2.setPower(0);
            fl.setVelocity(0);
            fr.setVelocity(0);
            rl.setVelocity(0);
            rr.setVelocity(0);
        }
        else {
            fl.setVelocity(0);
            fr.setVelocity(0);
            rl.setVelocity(0);
            rr.setVelocity(0);
            cs1.setPower(-0.1);
            cs2.setPower(0.1);
            sleep(4);
            cs1.setPower(0);
            cs2.setPower(0);
        }
        led.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
}