package org.firstinspires.ftc.teamcode.libs;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.HashMap;
import java.util.OptionalDouble;

public class CameraMain {
    private DcMotor[] motors;
    private double[] angles;
    private double[] motorSpeeds;
    private BNO055IMU imu;
    private HashMap<String, String> orientationModifier;
    private T265Camera camera;
    private AxesReference axesReference;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();
    private final Canvas field = packet.fieldOverlay();

    private Telemetry telemetry;

    // Create persistent variables
    boolean xComplete = false;
    boolean yComplete = false;
    boolean turnComplete = false;

    public void setUpInternal(DcMotor[] motors, double[] angles, HashMap<String, String> orientationModifiers, AxesReference axesReference, HardwareMap hardwareMap, Telemetry telemetry) {
        if (motors.length != angles.length) {
            throw new RuntimeException("Motor array length and angle array length are not the same! Check your code!");
        }
        this.motors = motors;
        this.motorSpeeds = new double[motors.length];
        this.angles = angles;
        setupIMU(hardwareMap);
        this.imu = getImu();
        this.orientationModifier = orientationModifiers;
        Pose2d startingPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
        setupCamera(hardwareMap, startingPose);
        this.camera = getCamera();
        this.axesReference = axesReference;
        this.telemetry = telemetry;
    }

    private Orientation gangles() {
        Orientation orientation = imu.getAngularOrientation(axesReference, AxesOrder.ZYX, AngleUnit.RADIANS);
        if (orientationModifier != null) {
            for (String k : this.orientationModifier.keySet()) {
                String v = this.orientationModifier.get(k);
                if (v == null) throw new IllegalArgumentException("Value cannot be null.");
                char axis = k.charAt(0);
                char action = k.charAt(2);
                switch (axis) {
                    case '1':
                        switch (action) {
                            case '+': orientation.firstAngle += Double.parseDouble(v); break;
                            case '-': orientation.firstAngle -= Double.parseDouble(v); break;
                            case '*': orientation.firstAngle *= Double.parseDouble(v); break;
                            case '/': orientation.firstAngle /= Double.parseDouble(v); break;
                            default: throw new IllegalArgumentException("Unknown operator " + action);
                        }
                        break;
                    case '2':
                        switch (action) {
                            case '+': orientation.secondAngle += Double.parseDouble(v); break;
                            case '-': orientation.secondAngle -= Double.parseDouble(v); break;
                            case '*': orientation.secondAngle *= Double.parseDouble(v); break;
                            case '/': orientation.secondAngle /= Double.parseDouble(v); break;
                            default: throw new IllegalArgumentException("Unknown operator " + action);
                        }
                        break;
                    case '3':
                        switch (action) {
                            case '+': orientation.thirdAngle += Double.parseDouble(v); break;
                            case '-': orientation.thirdAngle -= Double.parseDouble(v); break;
                            case '*': orientation.thirdAngle *= Double.parseDouble(v); break;
                            case '/': orientation.thirdAngle /= Double.parseDouble(v); break;
                            default: throw new IllegalArgumentException("Unknown operator " + action);
                        }
                        break;
                    default: throw new IllegalArgumentException("Unknown axis " + axis);
                }
            }
        }
        return orientation;
    }

    public void setPoseInternal(Pose2d pose) {
        this.camera.setPose(pose);
    }

    public boolean goToInternal(double moveX, double moveY, double theta, double speed) {
        // Wrap theta to localTheta
        double localTheta = wrap(theta);
        T265Camera.CameraUpdate up = this.camera.getLastReceivedCameraUpdate();
        if (up.confidence == T265Camera.PoseConfidence.Failed && telemetry != null) { telemetry.addLine("Failed"); telemetry.update(); return false; }

        Translation2d translation2d = up.pose.getTranslation();

        double currentX = translation2d.getX() / 0.0254;
        double currentY = translation2d.getY() / 0.0254;
        double currentTheta = gangles().firstAngle;

        double deltaX = moveX - currentX;
        double deltaY = moveY - currentY;
        double deltaTheta = wrap(localTheta - currentTheta);

        xComplete = abs(deltaX) < 0.2;
        yComplete = abs(deltaY) < 0.2;
        turnComplete = abs(deltaTheta) < 0.1;

        if (xComplete && yComplete && turnComplete) {
            stopWheel();
            return true;
        }

        double driveTheta = Math.atan2(yComplete ? 0 : -deltaY, xComplete ? 0 : deltaX);
        driveTheta += gangles().firstAngle;

        double localSpeed = speed;
        if (abs(deltaX) < 2 && abs(deltaY) < 2) {
            localSpeed *= avg(abs(deltaX), abs(deltaY)) / 24;
        } else
        if (abs(deltaX) < 5 && abs(deltaY) < 5) {
            localSpeed *= avg(abs(deltaX), abs(deltaY)) / 12;
        }
        localSpeed = clamp(0.2, 1, localSpeed);

        for (int i = 0; i < this.motors.length; i++) {
            double motorSpeed = (Math.sin(this.angles[i] - driveTheta) + clamp(-1, 1, deltaTheta)) * localSpeed;

            if (motorSpeed < 0.1 && motorSpeed > -0.1) { motorSpeed = 0; } else
            if (motorSpeed < 0.2 && motorSpeed > 0.1) { motorSpeed = 0.2; } else
            if (motorSpeed > -0.2 && motorSpeed < -0.1) { motorSpeed = -0.2; }

            this.motorSpeeds[i] = motorSpeed;
        }

        OptionalDouble optionalSpeed = Arrays.stream(motorSpeeds).max();
        double fastestSpeed = optionalSpeed.isPresent() ? optionalSpeed.getAsDouble() : 0;
        double maxSpeed = 0.8;
        boolean scale = fastestSpeed > maxSpeed;
        double scaleFactor = maxSpeed / fastestSpeed;
        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].setPower(scale ? this.motorSpeeds[i] * scaleFactor : this.motorSpeeds[i]);
        }


        final int robotRadius = 9;
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(currentX, currentY, robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = currentX + arrowX  / 2, y1 = currentY + arrowY / 2;
        double x2 = currentX + arrowX, y2 = currentY + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        packet.put("X", currentX);
        packet.put("Y", currentY);
        packet.put("Confidence", up.confidence);

        writeTelemetry(deltaX, deltaY, driveTheta);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        return false;
    }

    public Orientation getRotation() {
        return gangles();
    }

    public Translation2d getPosition() {
        Translation2d current = this.camera.getLastReceivedCameraUpdate().pose.getTranslation();
        return new Translation2d(current.getX() / 0.0254, current.getY() / 0.0254);
    }

    private void stopWheel() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(value, max));
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > PI) {
            if (newTheta < -PI) {
                newTheta += 2*PI;
            } else {
                newTheta -= 2*PI;
            }
        }
        return newTheta;
    }

    private double avg(double... inputs) {
        double output = 0;
        for (double input : inputs) {
            output += input;
        }
        output /= inputs.length;
        return output;
    }

    private void writeTelemetry(double deltaX, double deltaY, double driveTheta) {
        packet.put("FR Speed", motors[0].getPower());
        packet.put("RR Speed", motors[1].getPower());
        packet.put("RL Speed", motors[2].getPower());
        packet.put("FL Speed", motors[3].getPower());
        packet.put("Delta X", deltaX);
        packet.put("Delta Y", deltaY);
        packet.put("Drive Theta", driveTheta);
    }
}
