package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Dark Matter (-_-)")
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
    double angle = 0;
    double driveSpeed = 1.00;
    boolean runningInput = false;
    //Custom drive function:
    public void drive(double x2, double y2, double rx) {
        Orientation orientation = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double x = x2 * Math.cos(-(orientation.firstAngle - angle)) - y2 * Math.sin(-(orientation.firstAngle - angle));
        double y = y2 * Math.cos(-(orientation.firstAngle - angle)) - x2 * Math.sin(-(orientation.firstAngle - angle));
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
    //Test and Moves arm:
    public void arm() {
        if (gamepad2.left_stick_x > 0.1) {
            ar.setPower(Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y));
        }
        if (gamepad2.left_stick_x < -0.1) {
            ar.setPower(-Math.sqrt(gamepad2.left_stick_x * gamepad2.left_stick_x + gamepad2.left_stick_y * gamepad2.left_stick_y));
        }
        if (gamepad2.right_stick_x > 0.1) {
            ar.setPower(0.3);
        }
        if (gamepad2.right_stick_x < -0.1) {
            ar.setPower(-0.3);
        }
        if (gamepad2.left_stick_y > 0.1) {
            ap.setTargetPosition(-3050);
            ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ap.setPower(-Math.sqrt(gamepad2.left_stick_y * gamepad2.left_stick_y + gamepad2.left_stick_x * gamepad2.left_stick_x));
        }
        if (gamepad2.left_stick_y < -0.1) {
            ap.setTargetPosition(0);
            ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ap.setPower(Math.sqrt(gamepad2.left_stick_y * gamepad2.left_stick_y + gamepad2.left_stick_x * gamepad2.left_stick_x));
        }
        if (gamepad2.right_stick_y > 0.1) {
            ap.setTargetPosition(-3050);
            ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ap.setPower(-0.3);
        }
        if (gamepad2.right_stick_y < -0.1) {
            ap.setTargetPosition(0);
            ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ap.setPower(0.3);
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
        Gamepad cp1 = new Gamepad();
        Gamepad cp2 = new Gamepad();
        //While loop for OpMode
        while (opModeIsActive()) {
            //Checks if the arm is past boundaries
            if (ap.getCurrentPosition() < -5650) {
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                while (ap.getCurrentPosition() < -5500) {
                    ap.setPower(0.6);
                }
                ap.setPower(0);
            }
            //Updates and drives
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            drive(x2, y2, rx);
            //Turns 90 degrees Counterclockwise
            if (gamepad1.left_bumper) {
                double current = Globals.getImu().getAngularOrientation().firstAngle;
                while (Globals.getImu().getAngularOrientation().firstAngle < Globals.wrap(current - 90)) {
                    fl.setPower(-0.5);
                    fr.setPower(-0.5);
                    rl.setPower(0.5);
                    rr.setPower(0.5);
                }
                fl.setPower(0);
                fr.setPower(0);
                rl.setPower(0);
                rr.setPower(0);
            }
            //Turns 90 degrees Clockwise
            if (gamepad1.right_bumper) {
                double current = Globals.getImu().getAngularOrientation().firstAngle;
                while (Globals.getImu().getAngularOrientation().firstAngle < Globals.wrap(current + 90)) {
                    fl.setPower(0.5);
                    fr.setPower(0.5);
                    rl.setPower(-0.5);
                    rr.setPower(-0.5);
                }
                fl.setPower(0);
                fr.setPower(0);
                rl.setPower(0);
                rr.setPower(0);
            }
            //Arm up
            if (gamepad2.left_bumper) {
                ap.setTargetPosition(-3050);
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(-0.8);
            }
            //Arm down
            else if (gamepad2.left_trigger > 0.4) {
                ap.setTargetPosition(0);
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0.8);
            }
            //Disables arm movement up or down
            else {
                ap.setPower(0);
            }
            //Position 0 (Intake)
            if (gamepad2.a) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(0);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (Math.abs(ap.getCurrentPosition()) > 10) {
                    ap.setPower(-0.8);
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            //Position -3050 (Level 3 shipping hub)
            if (gamepad2.y) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(-3050);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Math.abs(ap.getCurrentPosition()) < 3040) {
                    while (Math.abs(ap.getCurrentPosition()) < 3040 && opModeIsActive()) {
                        ap.setPower(-0.8);
                    }
                }
                else if (Math.abs(ap.getCurrentPosition()) > 3060) {
                    while (Math.abs(ap.getCurrentPosition()) > 3060 && opModeIsActive()) {
                        ap.setPower(0.8);
                    }
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            //Position -2216 (Duck Position)
            if (gamepad2.x) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(-2116);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Math.abs(ap.getCurrentPosition()) < 2106) {
                    while (Math.abs(ap.getCurrentPosition()) < 2106 && opModeIsActive()) {
                        ap.setPower(-0.8);
                    }
                }
                else if (Math.abs(ap.getCurrentPosition()) > 2126) {
                    while (Math.abs(ap.getCurrentPosition()) > 2126 && opModeIsActive()) {
                        ap.setPower(0.8);
                    }
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            //Position -705 (Shared shipping hub)
            if (gamepad2.b) {
                ap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ap.setTargetPosition(-705);
                ap.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Math.abs(ap.getCurrentPosition()) < 695) {
                    while (Math.abs(ap.getCurrentPosition()) < 695 && opModeIsActive()) {
                        ap.setPower(-0.8);
                    }
                }
                else if (Math.abs(ap.getCurrentPosition()) > 715) {
                    while (Math.abs(ap.getCurrentPosition()) > 715 && opModeIsActive()) {
                        ap.setPower(0.8);
                    }
                }
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ap.setPower(0);
            }
            //Resets position 0
            if (ts2.isPressed()) {
                ap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ap.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            //Runs servos to output item
            if (gamepad2.right_trigger > 0.5) {
                cs1.setPower(-0.50);
                cs2.setPower(0.35);
            }
            //Runs servos to input item
            else if (!ts1.isPressed() && gamepad2.right_bumper) {
                cs1.setPower(0.50);
                cs2.setPower(-0.35);
                runningInput = true;
            }
            //If item presses button or arm driver presses right trigger
            else if (ts1.isPressed() || gamepad2.right_trigger > 0.2) {
                cs1.setPower(-0.50);
                cs2.setPower(0.35);
                runningInput = false;
                sleep(4);
                cs1.setPower(0);
                cs2.setPower(0);
            }
            // If none, sets power to 0
            else {
                cs1.setPower(0);
                cs2.setPower(0);
            }
            // Resets IMU angle
            if (gamepad1.left_stick_button) {
                angle = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            }
            // Drives forward
            if (gamepad1.dpad_up) {
                drive(0,0.9,0);
            }
            //Drives backward
            if (gamepad1.dpad_down) {
                drive(0,-0.9,0);
            }
            //Drives left
            if (gamepad1.dpad_left) {
                drive(0.9,0,0);
            }
            //Drives right
            if (gamepad1.dpad_right) {
                drive(-0.9,0,0);
            }
            // Turns left
            if (gamepad1.left_trigger > 0.4) {
                drive(0, 0, 0.8);
            }
            // Turns right
            if (gamepad1.right_trigger > 0.4) {
                drive(0, 0, -0.8);
            }
            //Ups turning speed
            if (gamepad1.a && !cp1.a) {
                turningspeed += 0.25;
                if (turningspeed == 1.25) {
                    turningspeed = 0.25;
                }
            }
            //Copy Gamepads
            try {
                cp1.copy(gamepad1);
                cp2.copy(gamepad2);
            } catch (Exception ignored) {}
            //Runs servos to input item
            if (runningInput) {
                cs1.setPower(0.50);
                cs2.setPower(-0.35);
            }
            arm();
        }
    }
}