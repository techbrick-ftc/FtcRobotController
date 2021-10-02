package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.libs.Globals.startCamera;
import static org.firstinspires.ftc.teamcode.libs.Globals.stopCamera;
import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.CameraTele;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;
import org.firstinspires.ftc.teamcode.libs.TestBot;

@TeleOp(name="",group="")
@Disabled
public class RunTeleCamera extends LinearOpMode implements TeleAuto {
    private final TestBot robot = new TestBot();
    private final FieldCentric drive = new FieldCentric();
    private final CameraTele auto = new CameraTele();
    private final double gamepadDeadzone = 0.05;
    private boolean doingAuto = false;

    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);

        DcMotor[] motors = {robot.frMotor(), robot.rrMotor(), robot.rlMotor(), robot.flMotor()};
        double[] angles = {PI/4, 3*PI/4, 5*PI/4, 7*PI/4};

        auto.setUp(motors, angles, null, AxesReference.EXTRINSIC, hardwareMap);
    
        waitForStart();
    
        // Pre-run
        Gamepad prev1 = new Gamepad();
        startCamera();

        while (opModeIsActive()) {
            if (!doingAuto) {
                drive.gyro();
                drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else {
                auto.Drive();
            }

            if (gamepad1.a && !prev1.a) {
                auto.setPosition(10, 10);
                doingAuto = true;
            }

            if (gamepad1.left_stick_x > gamepadDeadzone || gamepad1.left_stick_x < -gamepadDeadzone ||
                gamepad1.left_stick_y > gamepadDeadzone || gamepad1.left_stick_y < -gamepadDeadzone ||
                gamepad1.right_stick_x > gamepadDeadzone || gamepad1.right_stick_x < -gamepadDeadzone ||
                gamepad1.right_stick_y > gamepadDeadzone || gamepad1.right_stick_y < -gamepadDeadzone) {
                doingAuto = false;
            }

            try {
                prev1.copy(gamepad1);
            } catch (RobotCoreException ignored) {
                prev1 = new Gamepad();
            }
        }

        stopCamera();
    }
}
