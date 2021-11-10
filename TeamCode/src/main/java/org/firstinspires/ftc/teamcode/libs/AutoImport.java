// Our parent auto program, which is used in other auto programs.

package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.SimpleSlamra;

public class AutoImport extends LinearOpMode implements TeleAuto {
    // Defines vars
    protected DcMotor fr = null;
    protected DcMotor rr = null;
    protected DcMotor rl = null;
    protected DcMotor fl = null;
    //protected DcMotor armX = null;
    //protected DcMotor armY = null;
    protected CRServo spinner = null;

    //protected TouchSensor armBoundryMin = null;
    //protected TouchSensor armBoundryMax = null;

    protected SimpleSlamra slauto = new SimpleSlamra();
    protected EasyOpenCVImportable camera = new EasyOpenCVImportable();

    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();

    // vars used in program
    protected int activePosition;
    protected int startingPoseX;
    protected int startingPoseY;
    protected int camera1X;
    protected int camera1Y;
    protected int camera2X;
    protected int camera2Y;

    public AutoImport(int startX, int startY, int cam1X, int cam1Y, int cam2X, int cam2Y) {
        startingPoseX = startX;
        startingPoseY = startY;
        camera1X = cam1X;
        camera1Y = cam1Y;
        camera2X = cam2X;
        camera2Y = cam2Y;
    }

    public boolean driverAbort() {
        return false;
    }

    public void runOpMode() {
        // configures hardware
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        /*armX = hardwareMap.get(DcMotor.class, "armX");
        armX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armY = hardwareMap.get(DcMotor.class, "armY");
        armY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        spinner = hardwareMap.get(CRServo.class, "spinner");


        //armBoundaryMin = hardwareMap.get(TouchSensor.class, "armBoundaryMin");
        //armBoundaryMax = hardwareMap.get(TouchSensor.class, "armBoundaryMax");

        // initializes imu
        setupIMU(hardwareMap);
        telemetry.addLine("IMU Done");
        telemetry.update();

        // initializes easyopencv
        // width and height of vision box are hardcoded here
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, camera1X, camera1Y, camera2X, camera2Y, 18, 45);

        // initializes slamra
        Pose2d startingPose = new Pose2d(new Translation2d(startingPoseX * 0.0254, startingPoseY * 0.0254), new Rotation2d(0));
        setupCamera(hardwareMap, startingPose);
        sleep(5000);
        startCamera();

        telemetry.addLine("Cameras Done");
        telemetry.update();

        // adds start telemetry
        telemetry.addLine("hardware configured");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        // sets servos to starting positions
        //put code here

        camera.startDetection();

        while (!isStarted()) {
            // loops this until start is pressed
            packet.put("Position", activePosition);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Position", elementPosition(0));
            telemetry.update();
        }
        //camera.stopDetection();

        // passes hardware to slamra class
        DcMotor[] motors = {fr, rr, rl, fl};
        slauto.setUp(motors, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);
    }

    // Function which pushes the robot spinner into the wall, before running it. True = red
    public void doSpinny(boolean side, int timeout) {
        if (side) { // red
            slauto.drive(60, 60, -90, 0.5, timeout, this, false, false);
            spinner.setPower(0.5);
            sleep(5000);
            spinner.setPower(0);

        } else { // blue

        }
    }

    // Function which uses the webcam to return the team element's position
    public int elementPosition(long delay) {
        sleep(delay);
        return camera.getDetection();
    }

    // Function which sets encoder values to 0, and waits until they have reset
    public void resetEnc(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (motor.getCurrentPosition() != 0) {
            sleep(10);
        }
    }
}

