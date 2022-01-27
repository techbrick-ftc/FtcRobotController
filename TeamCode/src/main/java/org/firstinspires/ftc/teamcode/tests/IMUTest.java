package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp()
public class IMUTest extends LinearOpMode {
    private BNO055IMU imu;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(new BNO055IMU.Parameters());

        waitForStart();

        while (opModeIsActive()) {
            Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
            packet.put("X", angles.firstAngle);
            packet.put("Y", angles.secondAngle);
            packet.put("Z", angles.thirdAngle);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
