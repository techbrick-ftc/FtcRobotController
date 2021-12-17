package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="BLUE DROP OFF: LEFT")
public class DarkMatterAutoConsumer_DropOffBLUEleft extends LinearOpMode {
    //Global Variables
    //Ticks For Yaw: 2850 * angle / 360
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ar;
    DcMotorEx ap;
    CRServo cs1;
    CRServo cs2;
    Camera camera;
    DcMotorEx led;
    TouchSensor ts1;
    final double tpi_s = 46.5567;
    final double tpi_d = 43.0301;
    final int ticksHighPitch = -3350;
    final int ticksLowPitch = -800;
    final int ticksDegree90Yaw = -712;
    final int ticksDegree270Yaw = -2138;
    public void runInches(int inches, dirrection direct, double speed) {
        if (direct == dirrection.forward) {
            fl.setTargetPosition((int)(Math.round(tpi_d * inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_d * inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_d * inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_d * inches) + rr.getCurrentPosition()));
        }
        else if (direct == dirrection.backward) {
            fl.setTargetPosition((int)(Math.round(tpi_d * -inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_d * -inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_d * -inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_d * -inches) + rr.getCurrentPosition()));
        }
        else if (direct == dirrection.right) {
            fl.setTargetPosition((int)(Math.round(tpi_s * inches) + fl.getCurrentPosition()));
            fr.setTargetPosition((int)(Math.round(tpi_s * -inches) + fr.getCurrentPosition()));
            rl.setTargetPosition((int)(Math.round(tpi_s * -inches) + rl.getCurrentPosition()));
            rr.setTargetPosition((int)(Math.round(tpi_s * inches) + rr.getCurrentPosition()));
        }
        else if (direct == dirrection.left) {
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
        }
        else if (positionPitch == armPositionsPitch.output) {
            ap.setTargetPosition(ticksHighPitch);
        }
        else if (positionPitch == armPositionsPitch.low) {
            ap.setTargetPosition(ticksLowPitch);
        }
        else if (positionPitch == armPositionsPitch.current) {
            ap.setTargetPosition(ap.getCurrentPosition());
        }
        if (positionYaw == armPositionsYaw.start) {
            ar.setTargetPosition(0);
        }
        else if (positionYaw == armPositionsYaw.degree90) {
            ar.setTargetPosition(ticksDegree90Yaw);
        }
        else if (positionYaw == armPositionsYaw.degree270) {
            ar.setTargetPosition(ticksDegree270Yaw);
        }
        else if (positionYaw == armPositionsYaw.current) {
            ar.setTargetPosition(ar.getCurrentPosition());
        }
        ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ap.setVelocity(speedPitch);
        ar.setVelocity(speedYaw);
    }
    public void runOpMode() {
        //Setting variables
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        cs1 = hardwareMap.get(CRServo.class, "cs1");
        cs2 = hardwareMap.get(CRServo.class, "cs2");
        ts1 = hardwareMap.get(TouchSensor.class, "ts1");
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
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Injecting Dark Matter Automatically...");
        telemetry.update();
        //Waits for start of OpMode
        waitForStart();
        telemetry.addLine("Injecting Dark Matter Automatically...");
        telemetry.addLine("The world is automatically being consumed...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        runInches(32, dirrection.left, 750);
        sleep(500);
        armPos(armPositionsPitch.output, 1250, armPositionsYaw.current, 0);
        timerBreak.reset();
        while (timerBreak.seconds() < 3.5 || (!fl.isBusy() && !ap.isBusy() && !ar.isBusy())) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        armPos(armPositionsPitch.current, 0, armPositionsYaw.degree90, 1250);
        timerBreak.reset();
        while (timerBreak.seconds() < 2 || (!fl.isBusy() && !ap.isBusy() && !ar.isBusy())) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        cs1.setPower(-0.75);
        cs2.setPower(0.4);
        sleep(700);
        cs1.setPower(0);
        cs2.setPower(0);
        runInches(40, dirrection.right, 750);
        armPos(armPositionsPitch.current, 0, armPositionsYaw.degree270, 1250);
        timerBreak.reset();
        while (timerBreak.seconds() < 3.6 || (!fl.isBusy() && !ap.isBusy() && !ar.isBusy())) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        runInches(20, dirrection.backward, 750);
        armPos(armPositionsPitch.intake, 1250, armPositionsYaw.current, 0);
        while ((ar.isBusy() || ap.isBusy() || fl.isBusy() || fr.isBusy() || rl.isBusy() || rr.isBusy()) && opModeIsActive()) {
            telemetry.addData("FL", fl.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("RL", rl.getCurrentPosition());
            telemetry.addData("RR", rr.getCurrentPosition());
            telemetry.addData("AR", ar.getCurrentPosition());
            telemetry.addData("AP", ap.getCurrentPosition());
            telemetry.update();
        }
        cs1.setPower(0.75);
        cs2.setPower(-0.4);
        int prevCurrent = fl.getCurrentPosition();
        fl.setVelocity(-500);
        fr.setVelocity(-500);
        rl.setVelocity(-500);
        rr.setVelocity(-500);
        while (!(ts1.isPressed() || fl.getCurrentPosition() < prevCurrent - (20 * tpi_d)) || !opModeIsActive()) {
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
        cs1.setPower(-0.1);
        cs2.setPower(0.1);
        sleep(4);
        cs1.setPower(0);
        cs2.setPower(0);
    }
}