package org.firstinspires.ftc.teamcode.mains;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.libs.Nikolaj;

import static java.lang.Math.PI;

@Autonomous(name="",group="")
public class BlueCargo extends LinearOpMode {
    // Pre-init
    private final Nikolaj robot = new Nikolaj();
    private T265Camera slamera;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {
        // Init
        slamera = new T265Camera(
                new Transform2d(),
                0.1,
                hardwareMap.appContext);

        waitForStart();
    
        // Pre-run
        slamera.start();
        sleep(300);
        slamera.setPose(new Pose2d((2. * 0.0254), (-63. * 0.0254), new Rotation2d(0.)));

        slamera.getLastReceivedCameraUpdate();

        if (opModeIsActive()) {
            // Autonomous instructions
            while (opModeIsActive()) {
                final int robotRadius = 9; // inches

                TelemetryPacket packet = new TelemetryPacket();
                Canvas field = packet.fieldOverlay();

                T265Camera.CameraUpdate up = slamera.getLastReceivedCameraUpdate();
                if (up == null) return;

                // We divide by 0.0254 to convert meters to inches
                Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
                Rotation2d rotation = up.pose.getRotation();

                field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
                double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
                double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
                double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
                field.strokeLine(x1, y1, x2, y2);

                packet.put("X", translation.getX());
                packet.put("Y", translation.getY());
                packet.put("Angle", rotation.getRadians());

                dashboard.sendTelemetryPacket(packet);
            }
        }

        slamera.stop();
    }
}
