package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="SimpleSlamraDrive", group="test")
public class SimpleSlamraDrive extends AutoImport {

    public SimpleSlamraDrive() { super(0, 0, 225, 150, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            packet.addLine("one");
            dashboard.sendTelemetryPacket(packet);
            slauto.drive(0, 40, 0, 1, this);
            packet.addLine("two");
            dashboard.sendTelemetryPacket(packet);
            Globals.stopCamera();
        }
    }
}
