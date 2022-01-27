package org.firstinspires.ftc.teamcode.mains;


import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.CameraAuto;
import org.firstinspires.ftc.teamcode.libs.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.libs.Globals;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;

import static java.lang.Math.PI;

import java.util.HashMap;

@Autonomous()
public class BlueDuck extends LinearOpMode implements TeleAuto {
    private final Nikolaj robot = new Nikolaj();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket initPacket = new TelemetryPacket();
    private final EasyOpenCVImportable camera = new EasyOpenCVImportable();
    private final CameraAuto auto = new CameraAuto();

    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);

        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 250, 25, 250, 105, 20, 20);

        camera.startDetection();
        dashboard.startCameraStream(camera.getWebCamera(), 0);

        Globals.setupCamera(hardwareMap);

        auto.setUp(
                new DcMotor[]{robot.flMotor(), robot.frMotor(), robot.rrMotor(), robot.rlMotor()},
                new double[]{PI/4, 3*PI/4, 5*PI/4, 7*PI/4},
                null, AxesReference.EXTRINSIC, hardwareMap);

        ElapsedTime et = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Time in init", et.milliseconds() / 1000);
            telemetry.addData("Camera normalized", et.seconds() > 4);
            telemetry.update();

            initPacket.put("Time in init", et.milliseconds() / 1000);
            initPacket.put("Camera normalized", et.seconds() > 4);
            dashboard.sendTelemetryPacket(initPacket);
        }

        Globals.startCamera();
        sleep(400);
        Globals.setPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)));

        if (opModeIsActive()) {
            // TeleOp loop
            while (camera.getDetection() == -1 && opModeIsActive()) { idle(); }

            while (opModeIsActive() && !gamepad1.a) {
                final int robotRadius = 9; // inches
                TelemetryPacket packet = new TelemetryPacket();
                Canvas field = packet.fieldOverlay();

                T265Camera.CameraUpdate up = Globals.getCamera().getLastReceivedCameraUpdate();
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

            sleep(3000);

            auto.goToPosition(0, 21, this);
            auto.goToPosition(4, 21, this);
            auto.goTo(15, 21, PI/2, 0.5, this);
        }
        camera.stopDetection();
        Globals.stopCamera();
        Log.v("bobot","Freed camera");
    }
}
