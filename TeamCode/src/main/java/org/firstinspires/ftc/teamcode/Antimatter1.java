// Robot IP: 192.168.43.1/dash
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="DRIVE.EXE/Antimatter 1.0")
@Disabled
public class Antimatter1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor rl = hardwareMap.get(DcMotor.class, "rl");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rr");
        DcMotor[] motors = {fl, fr, rl, rr};
        double[] angles = {PI / 4, 3 * PI / 4, 5 * PI / 4, 7 * PI / 4};
        FieldCentric drive = new FieldCentric();
        final FtcDashboard ftcDashboard = FtcDashboard.getInstance();
        final TelemetryPacket telemetryPacket = new TelemetryPacket();

        Globals.setupIMU(hardwareMap);
//        drive.setUp(motors, angles);
        boolean halfSpeed = true;
        // Send telemetry message to signify robot waiting;
        telemetry.addLine("Loading...");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addLine("The War has Begun...");    //
        telemetry.update();
        Gamepad cp1 = new Gamepad();
        // run until the end of the match (driver presses STOP)
        int victoryDance = 0;
        boolean victoryDanceL = true;
        int debugOption = 1;
        fl.setDirection(DcMotorSimple.Direction.REVERSE); fr.setDirection(DcMotorSimple.Direction.REVERSE); rr.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive()) {
            while (victoryDance > 0) {
                if (victoryDance % 25000 == 0) {
                    victoryDanceL = !victoryDanceL;
                }
                drive.Drive(0, 0, victoryDanceL ? -0.75 : 0.75);
                victoryDance--;
            }
//            drive.gyro();
            drive.Drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, halfSpeed ? gamepad1.right_stick_x / 2 : gamepad1.right_stick_x);
            if (gamepad1.a && ! cp1.a) {
                halfSpeed = !halfSpeed;
                telemetry.addLine(halfSpeed ? "Half Speed Enabled." : "Half Speed Disabled.");
                telemetry.update();
            }

            if (gamepad1.b) {
                victoryDance = 200000;
            }
            if (gamepad1.y) {
                if (debugOption == 1) {
                    fl.setPower(1);
                }
                if (debugOption == 2) {
                    fr.setPower(1);
                }
                if (debugOption == 3) {
                    rr.setPower(1);
                }
                if (debugOption == 4) {
                    rl.setPower(1);
                }
            }
            if (!gamepad1.y && cp1.y) {
                debugOption += 1;
                if (debugOption == 5) {
                    debugOption = 1;
                }
            }
            try {
                cp1.copy(gamepad1);
            } catch (Exception ignored) {}
        }
    }
}

