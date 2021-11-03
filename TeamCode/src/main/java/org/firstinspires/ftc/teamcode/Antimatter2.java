package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;
@Disabled
@TeleOp(name="DRIVE.EXE/Antimatter 2.0")
public class Antimatter2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "fl");
        DcMotor fr = hardwareMap.get(DcMotor.class, "fr");
        DcMotor rl = hardwareMap.get(DcMotor.class, "rl");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rr");
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Loading...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("The War has Begun...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        double turningspeed = 1;
        while (opModeIsActive()) {
            Orientation orientation = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            Globals.getImu().getPosition();
            double y2 = -gamepad1.left_stick_y;
            double x2 = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x * turningspeed;
            double x = x2 * Math.cos(-orientation.firstAngle) - y2 * Math.sin(-orientation.firstAngle);
            double y = y2 * Math.cos(-orientation.firstAngle) - x2 * Math.sin(-orientation.firstAngle);
            telemetry.addLine("IMU Angles:" + orientation.firstAngle + " " + orientation.secondAngle + " " + orientation.thirdAngle);
            telemetry.addLine("X: " + x + "  |  Y: " + y);
            telemetry.update();
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fl.setPower(frontLeftPower);
            rl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            rr.setPower(backRightPower);
            if (gamepad1.a) {
                turningspeed += 0.25;
                if (turningspeed == 1.25) {
                    turningspeed = 0.25;
                }
            }
        }
    }
}

