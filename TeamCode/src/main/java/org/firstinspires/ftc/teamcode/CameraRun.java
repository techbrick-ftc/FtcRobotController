package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.Direction;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static org.firstinspires.ftc.teamcode.libs.Globals.startCamera;
import static org.firstinspires.ftc.teamcode.libs.Globals.stopCamera;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.CameraAuto;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;
import org.firstinspires.ftc.teamcode.libs.TestBot;

@Autonomous(name="Run Camera", group="Tests")
public class CameraRun extends LinearOpMode implements TeleAuto {
    private final CameraAuto cameraAuto = new CameraAuto();
    //private final TestBot robot = new TestBot();
    protected DcMotor m1 = null;
    protected DcMotor m2 = null;
    protected DcMotor m3 = null;
    protected DcMotor m4 = null;

    public void runOpMode() {
        //robot.setup(hardwareMap);
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

        DcMotor[] motors = {m1, m2, m3, m4};
        double[] angles = {3*PI/4, 5*PI/4, 7*PI/4, PI/4};

        cameraAuto.setUp(motors, angles, null, AxesReference.EXTRINSIC, hardwareMap, telemetry);

        waitForStart();

        startCamera();

        if (opModeIsActive()) {
            cameraAuto.goToPosition(10, 10, this);
            sleep(1000);
            cameraAuto.goToRotation(PI, this);
            sleep(1000);
            cameraAuto.goTo(5, 0, 0, 0.5, this);
        }

        stopCamera();
    }
}
