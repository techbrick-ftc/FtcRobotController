package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="",group="")
public class Barebons extends LinearOpMode {
    // Pre-init
    
    @Override
    public void runOpMode() {
        // Init
    
        waitForStart();
    
        // Pre-run
        final FtcDashboard dashboard = FtcDashboard.getInstance();
        final TelemetryPacket packet = new TelemetryPacket();
        int loops = 0;
        while (opModeIsActive()) {
            // TeleOp loop
            telemetry.addData("Loops", loops);
            telemetry.update();

            packet.put("Loops", loops);
            dashboard.sendTelemetryPacket(packet);

            loops++;

        }
    }
}
