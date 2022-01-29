package org.firstinspires.ftc.teamcode.libs;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

public class CameraAuto {
    private final CameraMain MAIN = new CameraMain();

    /**
     * Sets up internal variables for driving
     * @param motors Array of motors
     * @param angles Array of angles (referenced in same order as motors)
     * @param orientationModifiers Should you need to modify the IMU angles, make a key that is <samp>(axis# based on ZYX axis order):(operation)</samp> and a value that is the value for the operation. Otherwise, just pass <samp>null</samp>
     * @param axesReference Axes reference for getting the IMU angles
     * @param hardwareMap The hardware map from the op mode to be used for camera and imu set up.
     */
    public void setUp(DcMotor[] motors, double[] angles, HashMap<String, String> orientationModifiers, AxesReference axesReference, HardwareMap hardwareMap) {
        this.MAIN.setUpInternal(motors, angles, orientationModifiers, axesReference, hardwareMap, null);
    }

    /**
     * Sets up internal variables for driving
     * @param motors Array of motors
     * @param angles Array of angles (referenced in same order as motors)
     * @param orientationModifiers Should you need to modify the IMU angles, make a key that is <samp>(axis# based on ZYX axis order):(operation)</samp> and a value that is the value for the operation. Otherwise, just pass <samp>null</samp>
     * @param axesReference Axes reference for getting the IMU angles
     * @param hardwareMap The hardware map from the op mode to be used for camera and imu set up.
     * @param telemetry (Optional) Telemetry from OpMode for seeing details
     */
    public void setUp(DcMotor[] motors, double[] angles, HashMap<String, String> orientationModifiers, AxesReference axesReference, HardwareMap hardwareMap, Telemetry telemetry) {
        this.MAIN.setUpInternal(motors, angles, orientationModifiers, axesReference, hardwareMap, telemetry);
    }

    public void setPose(Pose2d pose) {
        this.MAIN.setPoseInternal(new Pose2d(pose.getTranslation().getX() * 0.0254, pose.getTranslation().getY() * 0.0254, pose.getRotation()));
    }

    /**
     * Drive to a position and hold the angle that the robot was at when it started.
     * <br>
     * <br>
     * FOR THE CALLBACK TO WORK, THE END OF YOUR CLASS LINE MUST BE <samp>implements TeleAuto</samp>
     * @param moveX The x position to move to.
     * @param moveY The y position to move to.
     * @param callback The program callback (typically just <samp>this</samp>) for using op mode
     *                 methods
     */
    public void goToPosition(double moveX, double moveY, TeleAuto callback) {
        goToPosition(moveX, moveY, 1, callback);
    }

    /**
     * Drive to a position and hold the angle that the robot was at when it started.
     * <br>
     * <br>
     * FOR THE CALLBACK TO WORK, THE END OF YOUR CLASS LINE MUST BE <samp>implements TeleAuto</samp>
     * @param moveX The x position to move to.
     * @param moveY The y position to move to.
     * @param speed A speed modifier (0.1 to 1) to slow the robot down, should you want to.
     * @param callback The program callback (typically just <samp>this</samp>) for using op mode
     *                 methods
     */
    public void goToPosition(double moveX, double moveY, double speed, @NotNull TeleAuto callback) {
        double angle = Globals.getAngle();

        goTo(moveX, moveY, angle, speed, callback);
    }

    /**
     * Spin the robot to a rotation and hold the robot at the position when it started.
     * <br>
     * <br>
     * FOR THE CALLBACK TO WORK, THE END OF YOUR CLASS LINE MUST BE <samp>implements TeleAuto</samp>
     * @param theta The theta (in radians) to spin the robot to.
     * @param callback The program callback (typically just <samp>this</samp>) for using op mode
     *                 methods
     */
    public void goToRotation(double theta, TeleAuto callback) {
        goToRotation(theta, 1, callback);
    }

    /**
     * Spin the robot to a rotation and hold the robot at the position when it started.
     * <br>
     * <br>
     * FOR THE CALLBACK TO WORK, THE END OF YOUR CLASS LINE MUST BE <samp>implements TeleAuto</samp>
     * @param theta The theta (in radians) to spin the robot to.
     * @param speed A speed modifier (0.1 to 1) to slow the robot down, should you want to.
     * @param callback The program callback (typically just <samp>this</samp>) for using op mode
     *                 methods
     */
    public void goToRotation(double theta, double speed, @NotNull TeleAuto callback) {
        Translation2d current = this.MAIN.getPosition();
        double currentX = current.getX();
        double currentY = current.getY();

        goTo(currentX, currentY, theta, speed, callback);
    }

    /**
     * Drive the robot to a position and a rotation.
     * <br>
     * <br>
     * FOR THE CALLBACK TO WORK, THE END OF YOUR CLASS LINE MUST BE <samp>implements TeleAuto</samp>
     * @param moveX The x position to move to.
     * @param moveY The y position to move to.
     * @param theta The theta (in radians) to spin the robot to.
     * @param callback The program callback (typically just <samp>this</samp>) for using op mode
     *                 methods
     */
    public void goTo(double moveX, double moveY, double theta, TeleAuto callback) {
        goTo(moveX, moveY, theta, 1, callback);
    }

    /**
     * Drive the robot to a position and a rotation.
     * <br>
     * <br>
     * FOR THE CALLBACK TO WORK, THE END OF YOUR CLASS LINE MUST BE <samp>implements TeleAuto</samp>
     * @param moveX The x position to move to.
     * @param moveY The y position to move to.
     * @param theta The theta (in radians) to spin the robot to.
     * @param speed A speed modifier (0.1 to 1) to slow the robot down, should you want to.
     * @param callback The program callback (typically just <samp>this</samp>) for using op mode
     *                 methods
     */
    public void goTo(double moveX, double moveY, double theta, double speed, @NotNull TeleAuto callback) {
        boolean complete;
        while (callback.opModeIsActive()) {
            complete = this.MAIN.goToInternal(moveX, moveY, theta, speed);
            if (complete) { return; }
        }
    }
}
