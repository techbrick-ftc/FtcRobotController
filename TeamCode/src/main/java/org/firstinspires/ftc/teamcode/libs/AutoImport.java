// Our parent auto program, which is used in other auto programs.

package org.firstinspires.ftc.teamcode.libs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    protected DcMotor m1 = null;
    protected DcMotor m2 = null;
    protected DcMotor m3 = null;
    protected DcMotor m4 = null;

    protected SimpleSlamra slauto = new SimpleSlamra();
    protected EasyOpenCVImportable camera = new EasyOpenCVImportable();
    protected BNO055IMU imu;

    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();

    // vars used in program
    protected int startingPoseX;
    protected int startingPoseY;
    protected int cameraX;
    protected int cameraY;

    public AutoImport(int startX, int startY, int camX, int camY) {
        startingPoseX = startX;
        startingPoseY = startY;
        cameraX = camX;
        cameraY = camY;
    }

    public boolean driverAbort() {
        return false;
    }

    public void runOpMode() {
        // configures hardware
        m4 = hardwareMap.get(DcMotor.class, "fl");
        m1 = hardwareMap.get(DcMotor.class, "fr");
        m3 = hardwareMap.get(DcMotor.class, "rl");
        m2 = hardwareMap.get(DcMotor.class, "rr");
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m1.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        // initializes imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(param);

        telemetry.addLine("IMU Done");
        telemetry.update();

        // initializes easyopencv
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, cameraX, cameraY, 45, 18);

        // initializes slamra
        Transform2d cameraToRobot = new Transform2d(new Translation2d(6 * 0.0254, 7 * 0.0254), Rotation2d.fromDegrees(-90));
        Pose2d startingPose = new Pose2d(new Translation2d(startingPoseX * 0.0254, startingPoseY * 0.0254), Rotation2d.fromDegrees(90));
        GlobalSlamra.startCamera(hardwareMap, cameraToRobot, startingPose);

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
        // loops this until start is pressed
        while (!isStarted()) {
            // you can put telemetry here
        }
        camera.stopDetection();

        // passes hardware to slamra class
        DcMotor[] motors = {m1, m2, m3, m4};
        slauto.setUp(motors, telemetry);

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);
    }

    // you can make functions in here to use in auto programs

    // Function which sets encoder values to 0, and waits until they have reset
    public void resetEnc(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (motor.getCurrentPosition() != 0) {
            sleep(10);
        }
    }
}

