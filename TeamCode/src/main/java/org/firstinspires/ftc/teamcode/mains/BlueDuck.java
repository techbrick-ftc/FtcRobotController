package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.libs.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;
import org.firstinspires.ftc.teamcode.tests.RobotConfig;

@Autonomous()
public class BlueDuck extends LinearOpMode {
    private final Nikolaj robot = new Nikolaj();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();
    private final EasyOpenCVImportable camera = new EasyOpenCVImportable();
    private T265Camera slamera;

    public void runOpMode() {
        // Init
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 250, 25, 250, 105, 20, 20);

        camera.startDetection();
        dashboard.startCameraStream(camera.getWebCamera(), 0);

        slamera = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);

        ElapsedTime et = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Time in init", et.milliseconds() / 1000);
            telemetry.addData("Camera normalized", et.seconds() > 4);
            telemetry.update();

            packet.put("Time in init", et.milliseconds() / 1000);
            packet.put("Camera normalized", et.seconds() > 4);
            dashboard.sendTelemetryPacket(packet);
        }

        slamera.start();
        slamera.setPose(new Pose2d((-35. * 0.0254), (-63. * 0.0254), new Rotation2d(0.)));

        if (opModeIsActive()) {
            // TeleOp loop
            while (camera.getDetection() == -1 && opModeIsActive()) {
                int[] analysis = camera.getAnalysis();
                int detection = camera.getDetection();

                packet.put("avg1 (blue)", analysis[0]);
                packet.put("avg2 (green)", analysis[1]);
                packet.put("Object Position", detection);
                dashboard.sendTelemetryPacket(packet);

                telemetry.addData("Object Position", detection);
                telemetry.update();
            }

            packet = new TelemetryPacket();

            while (opModeIsActive()) {
                final int robotRadius = 9; // inches
                Canvas field = packet.fieldOverlay();

                T265Camera.CameraUpdate up = slamera.getLastReceivedCameraUpdate();
                if (up == null) return;

                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = up.pose.getRotation();

                field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
                double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
                double x1 = translation.getX() + arrowX / 2, y1 = translation.getY() + arrowY / 2;
                double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
                field.strokeLine(x1, y1, x2, y2);

                packet.put("X", translation.getX());
                packet.put("Y", translation.getY());
                packet.put("Angle", rotation.getRadians());

                packet.put("Final pos", camera.getDetection());
                dashboard.sendTelemetryPacket(packet);
            }
        }
        camera.stopDetection();
        slamera.stop();
    }
}
