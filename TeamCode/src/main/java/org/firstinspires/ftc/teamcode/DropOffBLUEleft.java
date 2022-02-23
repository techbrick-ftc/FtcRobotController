package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="BLUE DROP OFF: LEFT")
public class DropOffBLUEleft extends LinearOpMode {
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
    final int ticksHighPitch = -3100;
    final int ticksMiddlePitch = -1600;
    final int ticksLowPitch = -800;
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
        if (positionPitch == armPositionsPitch.middle) {
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
            ap.setTargetPosition(-5000);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (positionPitch == armPositionsPitch.lvl1) {
            ap.setTargetPosition(-625);
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
        ap.setVelocity(speedPitch);
        ar.setVelocity(speedYaw);
    }
    public void armPosDegree(int positionPitch, int speedPitch, int positionYaw, int speedYaw) {
        if (positionPitch != 0) {
            ap.setTargetPosition(-18043 * positionPitch / 360);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(speedPitch);
        }
        if (positionYaw != 0) {
            ar.setTargetPosition(2850 * positionYaw / 360);
            ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ar.setVelocity(speedYaw);
        }
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
        ElapsedTime timerBreak = new ElapsedTime();
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
        runInches(24, direction.left, 900);
        sleep(200);
        armPos(armPositionsPitch.middle, 2500, armPositionsYaw.current, 0);
        sleep(300);
        armPosDegree(0, 0, -60, 1000);
        sleep(1500);
        if (!tsleft.isPressed() && !tsright.isPressed()) {
            runInches(6, direction.right, 800);
            armPos(armPositionsPitch.lvl1, 2200, armPositionsYaw.current, 0);
            sleep(1100);
            armPosDegree(0, 0, -124, 550);
            sleep(1000);
        }
        else if (tsleft.isPressed() && !tsright.isPressed()) {
            armPos(armPositionsPitch.middle, 2200, armPositionsYaw.current, 0);
            sleep(1100);
            armPosDegree(0, 0, -112, 820);
            sleep(1500);
        }
        else {
            armPos(armPositionsPitch.output, 2200, armPositionsYaw.current, 0);
            sleep(1100);
            armPosDegree(0, 0, -112, 820);
            sleep(1500);
        }
        cs1.setPower(-0.5);
        cs2.setPower(0.5);
        sleep(1300);
        cs1.setPower(0);
        cs2.setPower(0);
        if (tsleft.isPressed() || tsright.isPressed()) {
            runInches(35, direction.right, 920);
        }
        else {
            runInches(30, direction.right, 920);
        }
        sleep(700);
        armPos(armPositionsPitch.low, 2200, armPositionsYaw.degree270, 1000);
        timerBreak.reset();
        while (timerBreak.milliseconds() < 2200) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        runInches(17, direction.backward, 1000);
        armPos(armPositionsPitch.intake, 2000, armPositionsYaw.current, 0);
        timerBreak.reset();
        while (timerBreak.milliseconds() < 1500) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        fl.setVelocity(0);
        fr.setVelocity(0);
        rl.setVelocity(0);
        rr.setVelocity(0);
        cs1.setPower(0.55);
        cs2.setPower(-0.55);
        int prevCurrent = fl.getCurrentPosition();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setVelocity(-500);
        fr.setVelocity(-500);
        rl.setVelocity(-500);
        rr.setVelocity(-500);
        while (!ts1.isPressed() && fl.getCurrentPosition() > prevCurrent - (25 * tpi_d) && opModeIsActive()) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        if (ts1.isPressed()) {
            fl.setVelocity(0);
            fr.setVelocity(0);
            rl.setVelocity(0);
            rr.setVelocity(0);
            cs1.setPower(0);
            cs2.setPower(0);
            armPos(armPositionsPitch.output, 2200, armPositionsYaw.degree180, 800);
            sleep(100);
            runInches((int)Math.round((prevCurrent - fl.getCurrentPosition()) / tpi_d + 42), direction.forward, 1000);
            timerBreak.reset();
            while (timerBreak.milliseconds() < 3500) {
                telemetry.addData("FL", fl.getCurrentPosition());
                telemetry.addData("FR", fr.getCurrentPosition());
                telemetry.addData("RL", rl.getCurrentPosition());
                telemetry.addData("RR", rr.getCurrentPosition());
                telemetry.addData("AR", ar.getCurrentPosition());
                telemetry.addData("AP", ap.getCurrentPosition());
                telemetry.update();
            }
            runInches(16, direction.left, 1000);
            timerBreak.reset();
            while (timerBreak.milliseconds() < 1200) {
                telemetry.addData("FL", fl.getCurrentPosition());
                telemetry.addData("FR", fr.getCurrentPosition());
                telemetry.addData("RL", rl.getCurrentPosition());
                telemetry.addData("RR", rr.getCurrentPosition());
                telemetry.addData("AR", ar.getCurrentPosition());
                telemetry.addData("AP", ap.getCurrentPosition());
                telemetry.update();
            }
            cs1.setPower(-0.5);
            cs2.setPower(0.5);
            sleep(1000);
            cs1.setPower(0);
            cs2.setPower(0);
            runInches(21, direction.right, 1200);
            sleep(500);
            armPos(armPositionsPitch.up, 2200, armPositionsYaw.degree270, 1000);
            timerBreak.reset();
            while (timerBreak.milliseconds() < 1000) {
                telemetry.addData("FL", fl.getCurrentPosition());
                telemetry.addData("FR", fr.getCurrentPosition());
                telemetry.addData("RL", rl.getCurrentPosition());
                telemetry.addData("RR", rr.getCurrentPosition());
                telemetry.addData("AR", ar.getCurrentPosition());
                telemetry.addData("AP", ap.getCurrentPosition());
                telemetry.update();
            }
            runInches(54, direction.backward, 1200);
            timerBreak.reset();
            while (timerBreak.milliseconds() < 2500) {
                telemetry.addData("FL", fl.getCurrentPosition());
                telemetry.addData("FR", fr.getCurrentPosition());
                telemetry.addData("RL", rl.getCurrentPosition());
                telemetry.addData("RR", rr.getCurrentPosition());
                telemetry.addData("AR", ar.getCurrentPosition());
                telemetry.addData("AP", ap.getCurrentPosition());
                telemetry.update();
            }
            runInches(23, direction.left, 1000);
            sleep(1600);
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(100);
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
            armPos(armPositionsPitch.low, 2000, armPositionsYaw.current, 0);
        }
        led.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }
}