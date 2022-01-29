package org.firstinspires.ftc.teamcode.libs;

import static org.firstinspires.ftc.teamcode.libs.Globals.getAngle;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.HashMap;

public class CameraTele {
    private final CameraMain MAIN = new CameraMain();
    private double x = 0;
    private double y = 0;
    private double theta = 0;
    private double speed = 1;

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

    public void setPosition(double x, double y) { setPosition(x, y, 1); }

    public void setPosition(double x, double y, double speed) {
        this.x = x;
        this.y = y;
        this.theta = getAngle();
        this.speed = speed;
    }

    public void setRotation(double theta) { setRotation(theta, 1); }

    public void setRotation(double theta, double speed) {
        Translation2d position = this.MAIN.getPosition();
        this.x = position.getX();
        this.y = position.getY();
        this.theta = theta;
        this.speed = speed;
    }

    public void set(double x, double y, double theta) { set(x, y, theta, 1); }

    public void set(double x, double y, double theta, double speed) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.speed = speed;
    }

    public boolean Drive() {
        return this.MAIN.goToInternal(this.x, this.y, this.theta, this.speed);
    }
}
