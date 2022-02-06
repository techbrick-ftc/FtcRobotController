package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="SimpleSlamraDrive", group="test")
public class SimpleSlamraDrive extends AutoImport {

    public SimpleSlamraDrive() { super(0,  0, 0, 0, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            packet.addLine("Starting Travel");
            dashboard.sendTelemetryPacket(packet);
            slauto.drive(0, 15, 0, 0.5, this);
            packet.addLine("Arrived");
            dashboard.sendTelemetryPacket(packet);
        }
        Globals.endingPose = slauto.getPose();
        stopCamera();
    }
}
