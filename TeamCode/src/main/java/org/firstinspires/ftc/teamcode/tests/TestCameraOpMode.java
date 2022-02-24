package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import static java.lang.Math.PI;
import static java.lang.Thread.sleep;

import android.util.Log;

import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="Test T265", group="Test")
public class TestCameraOpMode extends OpMode {
    // We treat this like a singleton because there should only ever be one object per camera

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        Globals.setupCamera(hardwareMap);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        Globals.startCamera();
//        try {
//            sleep(3000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        while (Globals.getCamera().getLastReceivedCameraUpdate().pose.getX() == 0 &&
            Globals.getCamera().getLastReceivedCameraUpdate().pose.getY() == 0 &&
            Globals.getCamera().getLastReceivedCameraUpdate().pose.getRotation().getRadians() == 0)
            idle(); System.out.println("hi");
        Globals.setPose(new Pose2d(0.,10. * 0.0254,new Rotation2d(PI)));
    }

    private void idle() {
        try { sleep(1); } catch (Exception ignored) {}
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = Globals.getCamera().getLastReceivedCameraUpdate();
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

    @Override
    public void stop() {
        Globals.stopCamera();
    }

}