package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="DetectionTest", group="test")
public class DetectionTest extends AutoImport {

    public DetectionTest() {
        // change cam values
        super(0, 0, 75, 110, 200, 110);
    }

    public void runOpMode() {
        super.runOpMode();

        dashboard.startCameraStream(camera.getWebCamera(), 0);

        while (opModeIsActive()) {
            packet.put("status", camera.getDetecting());
            packet.put("detected", camera.getDetection());
            packet.put("analysis 1", camera.getAnalysis1());
            packet.put("analysis 2", camera.getAnalysis2());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}