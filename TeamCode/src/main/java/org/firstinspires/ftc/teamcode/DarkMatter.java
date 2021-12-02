package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Dark Matter 🙂")
public class DarkMatter extends LinearOpMode {
    //Global Variables
    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx rl;
    DcMotorEx rr;
    DcMotorEx ar;
    DcMotorEx ap;
    CRServo cs1;
    CRServo cs2;
    TouchSensor ts1;
    TouchSensor ts2;
    Camera camera;
    LED led;
    double angle = 0;
    double driveSpeed = 1.00;
    boolean runningInput = false;
    boolean duckControl = false;
    boolean pressed = false;
    boolean fieldCentric = true;
    boolean driveAllowed;
    Gamepad cp1;
    Gamepad cp2;
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

        fl.setVelocity((frontLeftPower * driveSpeed) * 1800);
        rl.setVelocity((backLeftPower * driveSpeed) * 1800);
        fr.setVelocity((frontRightPower * driveSpeed) * 1800);
        rr.setVelocity((backRightPower * driveSpeed) * 1800);
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
        double onx = gamepad2.right_stick_x;
        double onx2 = gamepad2.left_stick_x;
        if (onx > 0.3 && (ar.getCurrentPosition() < 0 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(1000);
        }
        else if (onx < -0.45 && ( ar.getCurrentPosition() > -9200 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ar.setVelocity(-1000);
        }
        else if (onx2 > 0.3 && ar.getCurrentPosition() < 0) {
            ar.setVelocity(Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y) * 2300);
        }
        else if (onx2 < -0.3 && ar.getCurrentPosition() > -9200) {
            ar.setVelocity(-Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y) * 2300);
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
            ap.setVelocity(-1600);
        }
        //Arm down
        else if (gamepad2.left_trigger > 0.4 && ( ap.getCurrentPosition() < 0 || gamepad2.left_stick_button || gamepad2.right_stick_button)) {
            ap.setVelocity(1600);
        }
        //Checks if not busy
        else if (!ap.isBusy()) {
            ap.setVelocity(0);
        }
        //Position 0 (Intake)
        if (gamepad2.a) {
            ap.setTargetPosition(0);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(-1600);
        }
        //Position -3050 (Level 3 shipping hub)
        else if (gamepad2.y) {
            ap.setTargetPosition(-3050);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(-1600);
        }
        //Position -2216 (Duck Position)
        else if (gamepad2.x) {
            ap.setTargetPosition(-2116);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(-1600);
        }
        //Position -705 (Shared shipping hub)
        else if (gamepad2.b) {
            ap.setTargetPosition(-705);
            ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ap.setVelocity(-1600);
        }
    }
    //
    //Input Output Servos Control
    //
    public void inputOutputControl() {
        //Duck Spin
        if (gamepad2.guide && !cp2.guide) {
            cs2.setPower(-1);
            duckControl = !duckControl;
        }
        //Runs servos to output item
        if (gamepad2.right_trigger > 0.5) {
            cs1.setPower(-0.55);
            cs2.setPower(0.35);
        }
        //Runs servos to input item
        else if (!ts1.isPressed() && gamepad2.right_bumper && !cp2.right_bumper) {
            cs1.setPower(0.50);
            cs2.setPower(-0.35);
            runningInput = !runningInput;
        }
        //If item presses button or arm driver presses right trigger
        else if (ts1.isPressed() || gamepad2.right_trigger > 0.2) {
            cs1.setPower(-0.50);
            cs2.setPower(0.35);
            runningInput = false;
            sleep(3);
            cs1.setPower(0);
            cs2.setPower(0);
        }
        // If none, sets power to 0
        else if (!runningInput) {
            cs1.setPower(0);
            cs2.setPower(0);
        }        //Runs servos to input item
        if (runningInput) {
            cs1.setPower(0.50);
            cs2.setPower(-0.35);
        }
        if (duckControl) {
            cs2.setPower(-1);
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
        cs1 = hardwareMap.get(CRServo.class, "cs1");
        cs2 = hardwareMap.get(CRServo.class, "cs2");
        ts1 = hardwareMap.get(TouchSensor.class, "ts1");
        ts2 = hardwareMap.get(TouchSensor.class, "ts2");
        ar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Injecting Dark Matter...");
        telemetry.update();
        //Waits for start of OpMode
        waitForStart();
        telemetry.addLine("Injecting Dark Matter...");
        telemetry.addLine("The world is consumed...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        double turningspeed = 1;
        cp1 = new Gamepad();
        cp2 = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();
        //While loop for OpMode
        while (opModeIsActive()) {
            //Updates and drives
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            driveAllowed = y2 > 0.2 || x2 > 0.2 || rx > 0.2 || y2 < -0.2 || x2 < -0.2 || rx < -0.2;
            if (driveAllowed) {
                if (fieldCentric) {
                    fieldCentricDrive(x2, y2, rx);
                }
                else {
                    robotCentricDrive(x2, y2, rx);
                }
            }
            //Calibrate pitch
            if (gamepad2.back) {
                ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
//            //Turns 90 degrees Counterclockwise
//            if (gamepad1.left_bumper) {
//                double current = Globals.getImu().getAngularOrientation().firstAngle;
//                while (Globals.getImu().getAngularOrientation().firstAngle > Globals.wrap(current - 90) && opModeIsActive()) {
//                    fl.setPower(0.3);
//                    fr.setPower(-0.3);
//                    rl.setPower(0.3);
//                    rr.setPower(-0.3);
//                }
//                fl.setPower(0);
//                fr.setPower(0);
//                rl.setPower(0);
//                rr.setPower(0);
//            }
//            //Turns 90 degrees Clockwise
//            if (gamepad1.right_bumper) {
//                timer1.reset();
//                while (Globals.getImu().getAngularOrientation().firstAngle < Globals.wrap(current + 90) && opModeIsActive() && timer1.seconds() < 5) {
//                    fl.setPower(-0.3);
//                    fr.setPower(0.3);
//                    rl.setPower(-0.3);
//                    rr.setPower(0.3);
//                }
//                fl.setPower(0);
//                fr.setPower(0);
//                rl.setPower(0);
//                rr.setPower(0);
//            }
            // Resets IMU angle
            if (gamepad1.left_stick_button) {
                angle = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
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
                fr.setVelocity(-1600);
                rl.setVelocity(1600);
                rr.setVelocity(1600);
            }
            //Drives right
            else if (gamepad1.dpad_right) {
                fl.setVelocity(1600);
                fr.setVelocity(1600);
                rl.setVelocity(-1600);
                rr.setVelocity(-1600);
            }
            else if (!driveAllowed) {
                fl.setVelocity(0);
                fr.setVelocity(0);
                rl.setVelocity(0);
                rr.setVelocity(0);
            }
            // Turns left
            if (gamepad1.left_trigger > 0.4) {
                fl.setPower(-0.8);
                fr.setPower(0.8);
                rl.setPower(-0.8);
                rr.setPower(0.8);
            }
            // Turns right
            if (gamepad1.right_trigger > 0.4) {
                fl.setPower(0.8);
                fr.setPower(-0.8);
                rl.setPower(0.8);
                rr.setPower(-0.8);
            }
            //Ups turning speed
            if (gamepad1.a && !cp1.a) {
                turningspeed += 0.25;
                if (turningspeed == 1.25) {
                    turningspeed = 0.25;
                }
            }
            //On or Off for FieldCentric
            if (gamepad1.b && !cp1.b) {
                fieldCentric = !fieldCentric;
            }
            //Copy Gamepads
            try {
                cp1.copy(gamepad1);
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
            armRotation();
            armPitch();
            inputOutputControl();
            if (ts2.isPressed()) {
                pressed = true;
            }

            //Resets position 0
            if (pressed && !ts2.isPressed()) {
                ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pressed = false;
            }
            //Something
            if (gamepad1.guide) {
                new ɿɘttɒMʞɿɒႧ();
            }
        }
    }
}