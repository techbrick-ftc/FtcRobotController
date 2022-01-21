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

@Autonomous()
public class BlueDuck extends LinearOpMode {
    private final Nikolaj robot = new Nikolaj();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket initPacket = new TelemetryPacket();
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

            initPacket.put("Time in init", et.milliseconds() / 1000);
            initPacket.put("Camera normalized", et.seconds() > 4);
            dashboard.sendTelemetryPacket(initPacket);
        }

        slamera.start();
        slamera.setPose(new Pose2d((-35.0 * 0.0254), (63.0 * 0.0254), new Rotation2d(0.0)));

        if (opModeIsActive()) {
            // TeleOp loop
            while (camera.getDetection() == -1 && opModeIsActive()) { idle(); }

            while (opModeIsActive()) {
                final int robotRadius = 9; // inches
                TelemetryPacket packet = new TelemetryPacket();
                Canvas field = packet.fieldOverlay();

                T265Camera.CameraUpdate up = slamera.getLastReceivedCameraUpdate();
                if (up == null) continue;

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

                telemetry.addData("X", translation.getX());
                telemetry.addData("Y", translation.getY());
                telemetry.addData("Angle", rotation.getRadians());
                telemetry.update();

                packet.put("Final pos", camera.getDetection());
                dashboard.sendTelemetryPacket(packet);
            }
        }
        camera.stopDetection();
        slamera.stop();
        slamera.free();
        telemetry.addLine("Freed camera");
        telemetry.update();
        System.out.println("Freed camera");
    }
}
