package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libs.Nikolaj;

@TeleOp(name="",group="")
public class EncoderStuff extends LinearOpMode {
    // Pre-init
    
    @Override
    public void runOpMode() {
        // Init
        Nikolaj robot = new Nikolaj();
        robot.setup(hardwareMap);

        robot.getLifter().setPower(0);
        robot.getLifter().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLifter().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.getArm().setPower(0);
        robot.getArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();
    
        // Pre-run
    
        while (opModeIsActive()) {
            // TeleOp loop
            packet.put("Lifter", robot.getLifter().getCurrentPosition());
            packet.put("Arm", robot.getArm().getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            robot.getLifter().setPower(gamepad1.left_stick_y);
        }
    }
}
