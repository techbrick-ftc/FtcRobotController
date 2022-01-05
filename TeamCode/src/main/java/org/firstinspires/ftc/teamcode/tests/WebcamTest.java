package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        easyOpenCVImportable.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, RobotConfig.box1x, RobotConfig.box1y, RobotConfig.box2x, RobotConfig.box2y, 20, 20);

        packet.put("x", RobotConfig.box1x);
        dashboard.sendTelemetryPacket(packet);

        easyOpenCVImportable.startDetection();
        dashboard.startCameraStream(easyOpenCVImportable.getWebCamera(), 0);

        ElapsedTime et = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Time in init", et.milliseconds() / 1000);
            telemetry.addData("Camera normalized", et.seconds() > 4);
            telemetry.update();

            packet.put("Time in init", et.milliseconds() / 1000);
            packet.put("Camera normalized", et.seconds() > 4);
            dashboard.sendTelemetryPacket(packet);
        }

        //TestBot bot = new TestBot();
        //bot.setup(hardwareMap);
        if (opModeIsActive()) {
            // TeleOp loop
            while (/*easyOpenCVImportable.getDetection() == -1 && */opModeIsActive()) {
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

