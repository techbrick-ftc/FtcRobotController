package org.firstinspires.ftc.teamcode.libs;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

/*
 * This file is made to be used in the CameraMain class so that the T265 camera and the internal
 * imu do not get instantiated more than once, because keeping them static means
 * THEY PRESERVE THEIR DATA!
 * Isn't that crazy! All this time we had to deal with the camera being reset to 0,0 or the imu
 * axis being reset all to 0, but with keeping that static, it doesn't happen!
 * If you wish to use the camera or imu in OpMode classes, just import and use the get methods and
 * assign that to a variable. For redundancy you can also run the setup methods, but the setup
 * methods are ran inside the CameraMain class, so unless your autonomous class doesn't use the
 * camera, it should already be setup!
 */

public class Globals {
    private static T265Camera camera;
    private static BNO055IMU imu;
    private static Transform2d transform = new Transform2d();

    public static void setCameraStart(Transform2d transform2d) {
        if (camera == null) {
            transform = transform2d;
        }
    }

    public static void setupCamera(HardwareMap hardwareMap) {
        if (camera == null) {
            camera = new T265Camera(
                    transform,
                    0.8,
                    hardwareMap.appContext
            );
        }
    }

    public static void setupIMU(HardwareMap hardwareMap) {
        if (imu == null) {
            imu = hardwareMap.get(BNO055IMU.class, "imu1");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            imu.initialize(params);
            System.out.println("Setup IMU");
        }
    }

    public static BNO055IMU getImu() { return imu; }
    public static T265Camera getCamera() { return camera; }

    public static void startCamera() { camera.start(); }
    public static void stopCamera() { camera.stop(); }
}
