package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.libs.TestBot;

@Autonomous(name="Webcam Test",group="")
public class WebcamTest extends LinearOpMode {
    private final EasyOpenCVImportable easyOpenCVImportable = new EasyOpenCVImportable();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {
        // Init
        easyOpenCVImportable.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 50, 50, 155, 50, 20, 20);

        easyOpenCVImportable.startDetection();
        dashboard.startCameraStream(easyOpenCVImportable.getWebCamera(), 0);
        waitForStart();
        TestBot bot = new TestBot();
        bot.setup(hardwareMap);
        if (opModeIsActive()) {
            // TeleOp loop
            while (easyOpenCVImportable.getDetection() == -1 && opModeIsActive()) {
                int[] analysis = easyOpenCVImportable.getAnalysis();
                int detection = easyOpenCVImportable.getDetection();

                packet.put("avg1 (blue)", analysis[0]);
                packet.put("avg2 (green)", analysis[1]);
                packet.put("Object Position", detection);
                dashboard.sendTelemetryPacket(packet);

                telemetry.addData("Object Position", detection);
                telemetry.update();
            }
        }
        easyOpenCVImportable.stopDetection();
    }
}
