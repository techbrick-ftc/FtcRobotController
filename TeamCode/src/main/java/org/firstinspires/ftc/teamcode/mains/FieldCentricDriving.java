package org.firstinspires.ftc.teamcode.mains;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;

@TeleOp(name="Main",group="")
public class FieldCentricDriving extends LinearOpMode {
    // Pre-init
    private final Nikolaj robot = new Nikolaj();
    private final FieldCentric drive = new FieldCentric();
    
    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);
        drive.setUp(
            new DcMotor[]{robot.frMotor(), robot.rrMotor(), robot.rlMotor(), robot.flMotor()},
            new double[]{PI/4, 3*PI/4, 5*PI/4, 7*PI/4},
            hardwareMap
        );
    
        waitForStart();
        AppUtil.getInstance().showToast(UILocation.BOTH, "Ethan's ego is bigger than the sun.");

        // Pre-run
        Gamepad cp1 = new Gamepad();
        while (opModeIsActive()) {
            // TeleOp loop
            drive.gyro();
            drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (robot.getLifter().getCurrentPosition() > -530 &&
                robot.getLifter().getCurrentPosition() < 0) {
                robot.getLifter().setPower(gamepad2.left_stick_y);
            }

            AppUtil.getInstance().getRootActivity().getActionBar().show();

            if (gamepad1.back && !cp1.back) { drive.resetAngle(); }

            try {
                cp1.copy(gamepad1);
            } catch (Exception ignored) {}
        }
    }
}
