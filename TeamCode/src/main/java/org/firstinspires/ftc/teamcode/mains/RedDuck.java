package org.firstinspires.ftc.teamcode.mains;

import static java.lang.Math.PI;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.CameraAuto;
import org.firstinspires.ftc.teamcode.libs.EasyOpenCVImportable;
import org.firstinspires.ftc.teamcode.libs.Globals;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;

@Autonomous
public class RedDuck extends LinearOpMode implements TeleAuto {
    // Pre-init
    private final Nikolaj robot = new Nikolaj();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket initPacket = new TelemetryPacket();
    private final EasyOpenCVImportable camera = new EasyOpenCVImportable();
    private final CameraAuto auto = new CameraAuto();

    int lifterDuck = 6943;
    
    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);

        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, 250, 40, 250, 120, 20, 20);

        camera.startDetection();
        dashboard.startCameraStream(camera.getWebCamera(), 0);

        Globals.setCameraStart(new Transform2d(new Translation2d(0., 0.), new Rotation2d(PI/2)));

        Globals.setupCamera(hardwareMap);
        Globals.setupIMU(hardwareMap);

        auto.setUp(
                new DcMotor[]{robot.frMotor(), robot.rrMotor(), robot.rlMotor(), robot.flMotor()},
                new double[]{PI/4, 3*PI/4, 5*PI/4, 7*PI/4},
                null, AxesReference.EXTRINSIC, hardwareMap);

        ElapsedTime et = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            int[] analysis = camera.getAnalysis();
            telemetry.addData("Time in init", et.milliseconds() / 1000);
            telemetry.addData("Camera normalized", (analysis[0] < 130 && analysis[1] < 130));
            telemetry.update();

            initPacket.put("Time in init", et.milliseconds() / 1000);
            initPacket.put("Camera normalized", (analysis[0] < 130 && analysis[1] < 130));
            initPacket.put("avg1", camera.getAnalysis()[0]);
            initPacket.put("avg2", camera.getAnalysis()[1]);
            dashboard.sendTelemetryPacket(initPacket);
        }

        Globals.startCamera();
        sleep(400);
        Globals.setPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)));

        if (opModeIsActive()) {
            // TeleOp loop
            while (camera.getDetection() == -1 && opModeIsActive()) { idle(); }
            int detection = camera.getDetection();
            camera.stopDetection();

            auto.goToPosition(0, -10, 0.3, this);

            auto.goToPosition(20, -10, 0.6, this);

            auto.goToRotation(-PI/2, 0.1, this);

            if (detection == 2) {
                robot.getLifter().setPower(1);
                while (robot.getLifter().getCurrentPosition() < 2634 && opModeIsActive()) idle();
                robot.getLifter().setPower(0);

                auto.goToPosition(19, -13, 0.2, this);

                robot.getLSrv().setPower(-1);
                robot.getRSrv().setPower(1);
                sleep(1000);
                robot.getLSrv().setPower(0);
                robot.getRSrv().setPower(0);
            } else if (detection == 1) {
                robot.getLifter().setPower(1);
                while (robot.getLifter().getCurrentPosition() < 5960 && opModeIsActive()) idle();
                robot.getLifter().setPower(0);

                auto.goToPosition(19, -14, 0.2, this);

                robot.getLSrv().setPower(-1);
                robot.getRSrv().setPower(1);
                sleep(1000);
                robot.getLSrv().setPower(0);
                robot.getRSrv().setPower(0);
            } else {
                robot.getLifter().setPower(1);
                while (robot.getLifter().getCurrentPosition() < 9895 && opModeIsActive()) idle();
                robot.getLifter().setPower(0);

                auto.goToPosition(19, -16, 0.2, this);

                robot.getLSrv().setPower(-1);
                robot.getRSrv().setPower(1);
                sleep(1000);
                robot.getLSrv().setPower(0);
                robot.getRSrv().setPower(0);
            }

            sleep(1000);
            auto.goTo(-31, -10, 0, 0.6, this);


            while (robot.getLifter().getCurrentPosition() > lifterDuck + 2 || robot.getLifter().getCurrentPosition() < lifterDuck - 2 && opModeIsActive()) {
                int lifterDel = lifterDuck - robot.getLifter().getCurrentPosition();
                robot.getLifter().setPower(Math.tanh(lifterDel/200.));
            }

            robot.getArm().setPower(0.3);
            robot.getArm().setTargetPosition(300);
            while (robot.getArm().getCurrentPosition() < 240 && opModeIsActive()) idle();
            ElapsedTime rampTime = new ElapsedTime();
            while (rampTime.seconds() < 3) { robot.getRSrv().setPower(-Math.tanh(3* rampTime.seconds())); }

            robot.getArm().setTargetPosition(0);
            auto.goToPosition(-31, -22, 0.7, this);
            while (robot.getArm().isBusy()) idle();
            robot.getLifter().setPower(-1);
            while (robot.getLifter().getCurrentPosition() > 0) idle();
            robot.getLifter().setPower(0);
        }
        camera.stopDetection();
        Globals.stopCamera();
        Log.v("bobot","Freed camera");
    }
}
