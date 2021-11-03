package org.firstinspires.ftc.teamcode.libs;

import android.content.Context;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    public static void setupCamera(HardwareMap hardwareMap) {
        System.out.println("***setting up t265");
        if (camera == null) {
            camera = new T265Camera(new Transform2d(new Translation2d(7 * 0.0254, 2 * 0.0254), new Rotation2d(0)), 0.1, hardwareMap.appContext);
            System.out.println("***finished setting up t265");
        }
    }

    public static void setupIMU(HardwareMap hardwareMap) {
        if (imu == null) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            imu.initialize(params);
        }
    }

    public static BNO055IMU getImu() { return imu; }
    public static T265Camera getCamera() { return camera; }

    public static void startCamera() {
        System.out.println("***starting t265");
        if (camera.isStarted()) {
            System.out.println("***t265 is already started, stopping");
            camera.stop();
        }
        camera.start();
        System.out.println("***started t256");
    }
    public static void stopCamera() {
        System.out.println("***stopping t265");
        camera.stop();
        System.out.println("***stopped t265");
    }
}
