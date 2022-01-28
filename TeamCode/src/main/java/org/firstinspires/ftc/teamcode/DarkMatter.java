package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Dark Matter")
public class DarkMatter extends LinearOpMode {
    //Global Variables
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ar;
    DcMotorEx duck;
    DcMotorEx ap;
    CRServo cs1;
    CRServo cs2;
    TouchSensor ts1;
    TouchSensor ts2;
    DcMotorEx led;
    double angle = Math.PI/2;
    double driveSpeed = 1.00;
    double turningspeed = 0.5;
    boolean runningInput = false;
    boolean pressed = false;
    boolean fieldCentric = true;
    boolean driveAllowed;
    boolean quacking = false;
    int quackingOn = 0;
    ElapsedTime quacker = new ElapsedTime();
    double y2;
    double x2;
    Gamepad cp1;
    Gamepad cp2;
    int yawOffSet = 0;
    //
    //FieldCentric Drive Control
    //
    public void fieldCentricDrive(double x2, double y2, double rx) {
        Orientation orientation = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double x = x2 * Math.cos(-(orientation.firstAngle - angle)) - y2 * Math.sin(-(orientation.firstAngle - angle));
        double y = y2 * Math.cos(-(orientation.firstAngle - angle)) + x2 * Math.sin(-(orientation.firstAngle - angle));
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.setVelocity((frontLeftPower * driveSpeed) * 2000);
        rl.setVelocity((backLeftPower * driveSpeed) * 2000);
        fr.setVelocity((frontRightPower * driveSpeed) * 2000);
        rr.setVelocity((backRightPower * driveSpeed) * 2000);
    }
    //
    //RobotCentric Drive Control
    //
    public void robotCentricDrive(double x2, double y2, double rx) {
        double denominator = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(rx), 1);
        double frontLeftPower = (y2 + x2 + rx) / denominator;
        double backLeftPower = (y2 - x2 + rx) / denominator;
        double frontRightPower = (y2 - x2 - rx) / denominator;
        double backRightPower = (y2 + x2 - rx) / denominator;

        fl.setVelocity((frontLeftPower * driveSpeed) * 1800);
        rl.setVelocity((backLeftPower * driveSpeed) * 1800);
        fr.setVelocity((frontRightPower * driveSpeed) * 1800);
        rr.setVelocity((backRightPower * driveSpeed) * 1800);
    }
    //
    //Arm Rotational Control
    //
    public void armRotation() {
        double onx = gamepad2.left_stick_x;
        if (onx > 0.05 && (ar.getCurrentPosition() < yawOffSet || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y) * 800);
        }
        else if (onx < -0.05 && (ar.getCurrentPosition() > -2336 + yawOffSet || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(-Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y) * 800);
        }
        else if (gamepad2.dpad_right && (ar.getCurrentPosition() < yawOffSet || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(200);
        }
        else if (gamepad2.dpad_left && (ar.getCurrentPosition() > -2336 + yawOffSet || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(-200);
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
        if (gamepad2.left_bumper && (ap.getCurrentPosition() > -5650 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(-2500);
        }
        //Arm down
        else if (gamepad2.left_trigger > 0.05 && ( ap.getCurrentPosition() < 0 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(1800);
        }
        //Arm up slow
        else if (gamepad2.dpad_up && (ap.getCurrentPosition() > -5650 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(-800);
        }
        //Arm down slow
        else if (gamepad2.dpad_down && ( ap.getCurrentPosition() < 0 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(800);
        }
        //Checks if not busy
        else if (!ap.isBusy()) {
            ap.setVelocity(0);
        }
        //Intake position
        if (gamepad2.a && !gamepad2.start && !gamepad1.start) {
            ap.setTargetPosition(0);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(2500);
            runningInput = true;
            cs1.setPower(0.55);
            cs2.setPower(-0.55);
        }
        //Level 3 shipping hub position
        else if (gamepad2.y) {
            ap.setTargetPosition(-3350);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(2500);
        }
        //Duck position
        else if (gamepad2.guide) {
            ap.setTargetPosition(-2022);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(2500);
        }
        //Shared shipping hub position
        else if (gamepad2.b && !gamepad2.start && !gamepad1.start) {
            ap.setTargetPosition(-875);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(2500);
        }
        //Capping position
        else if (gamepad2.x) {
            ap.setTargetPosition(-4100);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(2500);
        }
        if (!ap.isBusy()) {
            ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    //
    //Input Output Servos Control
    //
    public void inputOutputControl() {
        //Runs servos to output item
        if (gamepad2.right_trigger > 0.05) {
            cs1.setPower(-0.35);
            cs2.setPower(0.35);
            runningInput = false;
        }
        //Runs servos to input item
        else if (!ts1.isPressed() && gamepad2.right_bumper) {
            cs1.setPower(0.55);
            cs2.setPower(-0.55);
            runningInput = true;
        }
        // If none, sets power to 0
        else if (!runningInput) {
            cs1.setPower(0);
            cs2.setPower(0);
        }
        //If item presses button or arm driver presses right trigger
        else if (ts1.isPressed() && runningInput) {
            runningInput = false;
            sleep(4);
            cs1.setPower(0);
            cs2.setPower(0);
        }

    }
    public void quack(int quackerMode) {
        //Duck Spin
        if (!quacking) {
            quacker.reset();
            quackingOn = quackerMode;
            quacking = true;
            duck.setPower(0.5 * quackerMode);
        }
        if (quacker.seconds() > 0.2 && quacker.seconds() < 1.3) {
            duck.setPower(0.72 * quackerMode);
        }
        else if (quacker.seconds() > 1.3 && quacker.seconds() < 1.8) {
            duck.setPower(quackerMode);
        }
        else if (quacker.seconds() > 1.8) {
            quacking = false;
            duck.setPower(0);
        }
    }
    @Override
    //OpMode
    public void runOpMode() {
        //Setting variables and Init
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        rl = hardwareMap.get(DcMotorEx.class, "rl");
        rr = hardwareMap.get(DcMotorEx.class, "rr");
        ar = hardwareMap.get(DcMotorEx.class, "ar");
        ap = hardwareMap.get(DcMotorEx.class, "ap");
        duck = hardwareMap.get(DcMotorEx.class, "quack");
        cs1 = hardwareMap.get(CRServo.class, "cs1");
        cs2 = hardwareMap.get(CRServo.class, "cs2");
        ts1 = hardwareMap.get(TouchSensor.class, "ts1");
        ts2 = hardwareMap.get(TouchSensor.class, "ts2");
        led = hardwareMap.get(DcMotorEx.class, "B1");
        DcMotor[] all = {fl, fr, rl, rr, ap, ar, led};
        ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Waiting For Init...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        rl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Waits for start of OpMode
        waitForStart();
        cp1 = new Gamepad();
        cp2 = new Gamepad();
        ElapsedTime tm1 = new ElapsedTime();
        //While loop for OpMode
        while (opModeIsActive()) {
            //Telemetry
            telemetry.addLine("PITCH: " + ap.getCurrentPosition() + "; YAW: " + ar.getCurrentPosition());
            telemetry.addLine("fl: " + fl.getCurrentPosition() + "; fr: " + fr.getCurrentPosition() + ";" + "rl: " + rl.getCurrentPosition() + "; rr: " + rr.getCurrentPosition() + ";");
            telemetry.addLine("MODE: " + (fieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC"));
            telemetry.update();
            //Updates and drives
            Globals.getImu().getPosition();
            y2 = -gamepad1.left_stick_y;
            x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            driveAllowed = y2 > 0.05 || x2 > 0.05 || rx > 0.05 || y2 < -0.05 || x2 < -0.05 || rx < -0.05;
            if (driveAllowed) {
                if (fieldCentric) {
                    fieldCentricDrive(x2, y2, rx);
                }
                else {
                    robotCentricDrive(x2, y2, rx);
                }
            }
            //Duck spin
            if (gamepad1.b || gamepad1.x || quacking) {
                if (quacking) {
                    quack(quackingOn);
                }
                else {
                    quack(gamepad1.x ? 1 : -1);
                }

            }
            //Calibrate pitch
            if (gamepad2.back) {
                ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //Calibrate yaw
            if (gamepad2.start) {
                ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                yawOffSet = 1425;
            }
            //LED: CONTROL
            if (ts1.isPressed()) {
                if (led.getPower() < 0.5) {
                    led.setPower(1);
                    tm1.reset();
                }
            }
            else if (!ts1.isPressed()) {
                if (tm1.milliseconds() > 2500) {
                    led.setPower(0);
                }
            }
            // Resets IMU angle
            if (gamepad1.back) {
                angle = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + Math.PI/2;
                if (angle > Math.PI)
                {
                    angle -= 2*Math.PI;
                }

            }
            // Drives forward
            if (gamepad1.dpad_up) {
                fl.setVelocity(1600);
                fr.setVelocity(1600);
                rl.setVelocity(1600);
                rr.setVelocity(1600);
            }
            //Drives backward
            else if (gamepad1.dpad_down) {
                fl.setVelocity(-1600);
                fr.setVelocity(-1600);
                rl.setVelocity(-1600);
                rr.setVelocity(-1600);
            }
            //Drives left
            else if (gamepad1.dpad_left) {
                fl.setVelocity(-1600);
                fr.setVelocity(1600);
                rl.setVelocity(1600);
                rr.setVelocity(-1600);
            }
            //Drives right
            else if (gamepad1.dpad_right) {
                fl.setVelocity(1600);
                fr.setVelocity(-1600);
                rl.setVelocity(-1600);
                rr.setVelocity(1600);
            }
            else if (!driveAllowed) {
                fl.setVelocity(0);
                fr.setVelocity(0);
                rl.setVelocity(0);
                rr.setVelocity(0);
            }
            //Speeds
            if (gamepad1.right_bumper) {
                driveSpeed = 0.42;
                turningspeed = 0.32;
            }
            else {
                driveSpeed = 1;
                turningspeed = 0.5;
            }
            //Arm control
            armPitch();
            armRotation();
            inputOutputControl();
            if (ts2.isPressed()) pressed = true;
            //Resets position 0
            if (pressed && !ts2.isPressed()) {
                ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pressed = false;
            }
            //Field Centric
            if (gamepad1.guide && !cp1.guide) {
                fieldCentric = !fieldCentric;
            }
            //Copy Gamepads
            try {
                cp1.copy(gamepad1);
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
            idle();
        }
        for (DcMotor motor : all) {
            motor.setPower(0);
        }
        cs1.setPower(0);
        cs2.setPower(0);
    }
}