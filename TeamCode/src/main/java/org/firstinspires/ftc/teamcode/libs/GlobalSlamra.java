// We handle the T265 as a singleton, making it possible to access between op mode runs without a
// robot reset.

package org.firstinspires.ftc.teamcode.libs;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

public class GlobalSlamra {
    private static T265Camera camera = null;

    public static void startCamera(HardwareMap hardwareMap, Transform2d transform2d, Pose2d pose2d) {
        if (camera == null) {
            camera = new T265Camera(transform2d, 0.1, hardwareMap.appContext);
            try {
                Thread.sleep(2000);
            } catch (Exception ignored) {}
            camera.setPose(pose2d);
            camera.start();
        }
    }

    public static T265Camera.CameraUpdate getUpdate() {
        return camera.getLastReceivedCameraUpdate();
    }

    public static void setPose(Pose2d pose2d) {
        camera.setPose(pose2d);
    }
}