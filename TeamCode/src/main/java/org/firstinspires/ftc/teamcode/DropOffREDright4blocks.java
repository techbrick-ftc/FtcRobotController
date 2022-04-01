package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="RED DROP OFF: RIGHT 4 blocks")
public class DropOffREDright4blocks extends LinearOpMode {
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
    ColorSensor cs;
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
        if (speed < 1400) {
            setSpeed((int)speed);
        }
        else
            speedUp(speed);
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
    public void speedUp(double maxSpeed) {
        double topSpeed = maxSpeed + 125;
        fl.setVelocity(topSpeed / 2);
        fr.setVelocity(topSpeed / 2);
        rl.setVelocity(topSpeed / 2);
        rr.setVelocity(topSpeed / 2);
        sleep(250);
        fl.setVelocity(topSpeed - 75);
        fr.setVelocity(topSpeed - 75);
        rl.setVelocity(topSpeed - 75);
        rr.setVelocity(topSpeed - 75);
    }
    public void setSpeed(int speed) {
        fl.setVelocity(speed);
        fr.setVelocity(speed);
        rl.setVelocity(speed);
        rr.setVelocity(speed);
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
        cs = hardwareMap.get(ColorSensor.class, "colorSense");
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
        telemetry.addLine("Injecting Dark Matter Automatically... Version: 1.0.0");
        telemetry.update();
//Waits for start of OpMode
        waitForStart();
        if (hecks.errorStop) {
            stop();
        }
        telemetry.addLine("Injecting Dark Matter Automatically...");
        telemetry.addLine("The world is automatically being consumed...");
        telemetry.update();
        led.setPower(1);
        runInches(24, direction.left, 1390);
        sleep(100);
        armPos(armPositionsPitch.middle, 2500, armPositionsYaw.current, 0);
        sleep(200);
        armPosDegree(0, 0, -195, 1700);
        sleep(1200);
        toEncoder();
        if (tsleft.isPressed() && !tsright.isPressed()) {
            runInches(6, direction.right, 1200);
            armPos(armPositionsPitch.lvl1, 2500, armPositionsYaw.current, 0);
            sleep(500);
            armPosDegree(0, 0, -236, 800);
            sleep(400);
        }
        else if (!tsleft.isPressed() && tsright.isPressed()) {
            armPos(armPositionsPitch.middle, 2200, armPositionsYaw.current, 0);
            sleep(500);
            armPosDegree(0, 0, -248, 900);
            sleep(400);
        }
        else {
            armPos(armPositionsPitch.output, 2500, armPositionsYaw.current, 0);
            sleep(500);
            armPosDegree(0, 0, -248, 900);
            sleep(400);
        }
        toEncoder();
        cs1.setPower(-0.6);
        cs2.setPower(0.6);
        sleep(500);
        cs1.setPower(0);
        cs2.setPower(0);
//PICKUP #1
        if (tsleft.isPressed() && !tsright.isPressed()) {
            runInches(28, direction.right, 1390);
        }
        else {
            runInches(34, direction.right, 1390);
        }
        sleep(600);
        armPos(armPositionsPitch.low, 900, armPositionsYaw.current, 0);
        armPosDegree(0, 0, -80, 1500);
        sleep(1000);
        toEncoder();
        runInches(29, direction.forward, 1200);
        armPos(armPositionsPitch.intake, 2500, armPositionsYaw.current, 0);
        cs1.setPower(1);
        cs2.setPower(-1);
        boolean alreadyDone = false;
        while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy() && !alreadyDone) {
            if (cs.red() + cs.green() + cs.blue() > 900) {
                led.setPower(0);
                runInches(6, direction.forward, 1200);
                telemetry.addLine("DETECTED! Pickup 1 Is Almost Done."); telemetry.update();
                alreadyDone = true;
            }
        }
        while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
        led.setPower(1);
        toEncoder();
        int prevCurrent = fl.getCurrentPosition();
        fl.setVelocity(500);
        fr.setVelocity(500);
        rl.setVelocity(500);
        rr.setVelocity(500);
        while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (23 * tpi_d) && opModeIsActive()) {
        }
        if (fl.getCurrentPosition() < prevCurrent + (23 * tpi_d)) {
            fl.setVelocity(0);
            fr.setVelocity(0);
            rl.setVelocity(0);
            rr.setVelocity(0);
            cs1.setPower(0);
            cs2.setPower(0);
            armPos(armPositionsPitch.output, 2500, armPositionsYaw.current, 0);
            sleep(250);
            armPosDegree(0, 0, -225, 800);
            runInches((int) Math.round((fl.getCurrentPosition() - prevCurrent) / tpi_d + 32), direction.backward, 2450);
            while (fl.getCurrentPosition() > fl.getTargetPosition() + (16 * tpi_d)) {}
            setSpeed(1150);
            led.setPower(0);
            while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
            led.setPower(1);
            toEncoder();
            fl.setVelocity(2450);
            fr.setVelocity(-2450);
            rl.setVelocity(-2450);
            rr.setVelocity(2450);
            sleep(200);
            toEncoder();
            runInches(21, direction.left, 1390);
            sleep(1050);
            toEncoder();
            cs1.setPower(-0.6);
            cs2.setPower(0.6);
            sleep(500);
            cs1.setPower(0);
            cs2.setPower(0);
//Pickup #2
            runInches(25, direction.right, 1390);
            sleep(400);
            armPos(armPositionsPitch.low, 1400, armPositionsYaw.current, 0);
            armPosDegree(0, 0, -84, 1500);
            sleep(1000);
            toEncoder();
            runInches(36, direction.forward, 1200);
            armPos(armPositionsPitch.intake, 2500, armPositionsYaw.current, 0);
            cs1.setPower(1);
            cs2.setPower(-1);
            alreadyDone = false;
            while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy() && !alreadyDone) {
                if (cs.red() + cs.green() + cs.blue() > 900) {
                    led.setPower(0);
                    armPosDegree(0, 0, -90, 1600);
                    runInches(6, direction.forward, 1200);
                    telemetry.addLine("DETECTED! Pickup 2 Is Almost Done."); telemetry.update();
                    alreadyDone = true;
                }
            }
            while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
            armPosDegree(0, 0, -90, 1600);
            led.setPower(1);
            toEncoder();
            prevCurrent = fl.getCurrentPosition();
            fl.setVelocity(500);
            fr.setVelocity(500);
            rl.setVelocity(500);
            rr.setVelocity(500);

            while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (25 * tpi_d) && opModeIsActive()) {
            }
            if (fl.getCurrentPosition() < prevCurrent + 25 * tpi_d) {
                fl.setVelocity(0);
                fr.setVelocity(0);
                rl.setVelocity(0);
                rr.setVelocity(0);
                cs1.setPower(0);
                cs2.setPower(0);
                armPos(armPositionsPitch.output, 2500, armPositionsYaw.current, 0);
                sleep(250);
                armPosDegree(0, 0, -225, 800);
                runInches((int) Math.round((fl.getCurrentPosition() - prevCurrent) / tpi_d + 32), direction.backward, 2450);
                while (fl.getCurrentPosition() > fl.getTargetPosition() + (16 * tpi_d)) {}
                led.setPower(0);
                setSpeed(1150);
                while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {
                }
                led.setPower(1);
                toEncoder();
                fl.setVelocity(2450);
                fr.setVelocity(-2450);
                rl.setVelocity(-2450);
                rr.setVelocity(2450);
                sleep(200);
                toEncoder();
                runInches(21, direction.left, 1390);
                sleep(1050);
                toEncoder();
                cs1.setPower(-0.6);
                cs2.setPower(0.6);
                sleep(500);
                cs1.setPower(0);
                cs2.setPower(0);
//Pickup #3
                runInches(25, direction.right, 1390);
                sleep(400);
                armPos(armPositionsPitch.low, 1400, armPositionsYaw.current, 0);
                armPosDegree(0, 0, -84, 1500);
                cs1.setPower(1);
                cs2.setPower(-1);
                sleep(1000);
                toEncoder();
                runInches(36, direction.forward, 1200);
                armPos(armPositionsPitch.intake, 2500, armPositionsYaw.degree90, 1000);
                alreadyDone = false;
                while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy() && !alreadyDone) {
                    if (cs.red() + cs.green() + cs.blue() > 900) {
                        led.setPower(0);
                        armPosDegree(0, 0, -105, 1600);
                        runInches(6, direction.forward, 1200);
                        telemetry.addLine("DETECTED! Pickup 3 Is Almost Done."); telemetry.update();
                        alreadyDone = true;
                    }
                }
                while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
                armPosDegree(0, 0, -105, 1600);
                led.setPower(1);
                toEncoder();
                prevCurrent = fl.getCurrentPosition();
                fl.setVelocity(500);
                fr.setVelocity(500);
                rl.setVelocity(500);
                rr.setVelocity(500);
                while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (25 * tpi_d) && opModeIsActive()) {
                }
                if (fl.getCurrentPosition() < prevCurrent + (25 * tpi_d)) {
                    fl.setVelocity(0);
                    fr.setVelocity(0);
                    rl.setVelocity(0);
                    rr.setVelocity(0);
                    cs1.setPower(0);
                    cs2.setPower(0);
                    armPos(armPositionsPitch.output, 2500, armPositionsYaw.current, 0);
                    sleep(250);
                    armPosDegree(0, 0, -225, 800);
                    runInches((int) Math.round((fl.getCurrentPosition() - prevCurrent) / tpi_d + 32), direction.backward, 2450);
                    while (fl.getCurrentPosition() > fl.getTargetPosition() + (16 * tpi_d)) {
                    }
                    led.setPower(0);
                    setSpeed(1150);
                    while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {
                    }
                    led.setPower(1);
                    toEncoder();
                    fl.setVelocity(2450);
                    fr.setVelocity(-2450);
                    rl.setVelocity(-2450);
                    rr.setVelocity(2450);
                    sleep(200);
                    toEncoder();
                    runInches(21, direction.left, 1390);
                    sleep(1050);
                    toEncoder();
                    cs1.setPower(-0.6);
                    cs2.setPower(0.6);
                    sleep(500);
                    cs1.setPower(0);
                    cs2.setPower(0);
                    //Park
                    runInches(25, direction.right, 1390);
                    sleep(300);
                    armPos(armPositionsPitch.low, 900, armPositionsYaw.current, 0);
                    armPosDegree(0, 0, -70, 1500);
                    sleep(1100);
                    toEncoder();
                    runInches(33, direction.forward, 2400);
                    armPos(armPositionsPitch.intake, 2200, armPositionsYaw.current, 0);
                    cs1.setPower(1);
                    cs2.setPower(-1);
                    while (fl.isBusy() && fr.isBusy() && rl.isBusy() && rr.isBusy()) {}
                    toEncoder();
                    fl.setVelocity(500);
                    fr.setVelocity(500);
                    rl.setVelocity(500);
                    rr.setVelocity(500);
                    while (!ts1.isPressed() && fl.getCurrentPosition() < prevCurrent + (21 * tpi_d) && opModeIsActive()) {}
                    cs1.setPower(0);
                    cs2.setPower(0);
                    fl.setVelocity(0);
                    fr.setVelocity(0);
                    rl.setVelocity(0);
                    rr.setVelocity(0);
                }
            }
        }
        fl.setVelocity(0);
        fr.setVelocity(0);
        rl.setVelocity(0);
        rr.setVelocity(0);
        cs1.setPower(0);
        cs2.setPower(0);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        led.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);

    }
}