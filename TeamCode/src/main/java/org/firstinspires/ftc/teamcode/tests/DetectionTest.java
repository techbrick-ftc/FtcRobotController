package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Disabled
@Autonomous(name="DetectionTest", group="test")
public class DetectionTest extends AutoImport {

    public DetectionTest() {
        // change cam values
        super(0, 0, 35, 172, 135, 172);
    }

    public void runOpMode() {
        super.runOpMode();

        camera.startDetection();
        dashboard.startCameraStream(camera.getWebCamera(), 0);

        while (opModeIsActive()) {
            packet.put("status", camera.getDetecting());
            packet.put("detected", camera.getDetection());
            packet.put("analysis", camera.getAnalysis());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}