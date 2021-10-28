package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.Globals;
//535 tpr
@TeleOp(name="AFK ANTIMATTER MACHINE")
public class AFK_ANTIMATTER_MACHINE extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor rl;
    DcMotor rr;
    final double ticksPerInch = 1000;
    public void afkdriveTURBO(double inX) {
        afkdriveSPEEDSET(inX, 1);
    }
    public void afkdriveSPEEDSET(double inX, double speed) {
        fl.setTargetPosition((int)(inX * ticksPerInch) + fl.getCurrentPosition());
        fr.setTargetPosition((int)(inX * ticksPerInch) + fr.getCurrentPosition());
        rl.setTargetPosition((int)(inX * ticksPerInch) + rl.getCurrentPosition());
        rr.setTargetPosition((int)(inX * ticksPerInch) + rr.getCurrentPosition());
        fl.setPower(speed);
        fr.setPower(speed);
        rl.setPower(speed);
        rr.setPower(speed);
    }
    public void turn(double degree) {
        Orientation orientation = Globals.getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (degree > orientation.firstAngle) {
            while (orientation.firstAngle < degree) {
                fl.setPower(Math.PI - 3 + 0.3);
                fr.setPower(-(Math.PI - 3 + 0.3));
                rl.setPower(Math.PI - 3 + 0.3);
                rr.setPower(-(Math.PI - 3 + 0.3));
            }
        }
        else {
            while (orientation.firstAngle > degree) {
                fl.setPower(-(Math.PI - 3 + 0.3));
                fr.setPower(Math.PI - 3 + 0.3);
                rl.setPower(-(Math.PI - 3 + 0.3));
                rr.setPower(Math.PI - 3 + 0.3);
            }
        }
    }
    public void DEBUG() {
        telemetry.addData("ENCODER.TICKS", fl.getCurrentPosition());
        telemetry.addData("ENCODER.TICKS", fr.getCurrentPosition());
        telemetry.addData("ENCODER.TICKS", rl.getCurrentPosition());
        telemetry.addData("ENCODER.TICKS", rr.getCurrentPosition());
        telemetry.update();
    }
        @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        Globals.setupIMU(hardwareMap);
        telemetry.addLine("Loading...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("The AFK Minecraft Player has started AFKing...");
        telemetry.update();
        fl.setDirection(DcMotorSimple.Direction.REVERSE); rl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()){
            DEBUG();
            if (gamepad1.a) {
                fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                fl.setTargetPosition(535 + fl.getCurrentPosition());
                fr.setTargetPosition(535 + fr.getCurrentPosition());
                rl.setTargetPosition(535 + rl.getCurrentPosition());
                rr.setTargetPosition(535 + rr.getCurrentPosition());
                fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                fl.setPower(0.8);
                fr.setPower(0.8);
                rl.setPower(0.8);
                rr.setPower(0.8);

            }
        }
    }
}