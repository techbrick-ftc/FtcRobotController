package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="MainTele", group="teleop")
public class MainTele extends AutoImport {

    public MainTele() { super(31, -56, 225, 150, 255, 150); }
    FieldCentric drive = new FieldCentric();
    public boolean driverAbort() {
        return gamepad1.y;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        int loops = 0;

        // adds start telemetry
        telemetry.addLine("hardware ready");
        telemetry.update();

        // Defines motor configs
        final double PI = Math.PI;
        DcMotor[] motors = {fr, rr, rl, fl};
        double[] motorAngles = {3*PI/4, 5*PI/4, 7*PI/4, PI/4};

        // Sets up motor configs
        try {
            drive.setUp(motors, motorAngles);
        } catch (Exception e) {
            packet.put("SETUP ERROR", true);
            dashboard.sendTelemetryPacket(packet);
            e.printStackTrace();
        }

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Set up variables
        boolean intaking = false;
        boolean outtaking = false;
        boolean isSpinningForth = false;
        boolean isSpinningBack = false;
        boolean hatchOpen = false;
        double armXMin = -0.5;
        double armXMax = 0.5;
        int armYSetting = 0;
        int[] armYPositions = {armY.getCurrentPosition(), -663, -1692, -2590};

        // Starting servo & motor positions

        waitForStart();
    
        while (opModeIsActive()) {
            // Updates cur1 & 2
            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Gives FieldCentric the stick positions
            drive.Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);

            // Controls arm horizontal axis
            // Enforces encoder barriers at 90 and -90 degrees of starting position horizontally
            if (armX.getCurrentPosition() >= 926) { // constant is ~1/4 of full encoder rotation
                armXMax = 0;
                armXMin = -0.5;
            } else if (armX.getCurrentPosition() <= -926) {
                armXMax = 0.5;
                armXMin = 0;
            } else {
                armXMax = 0.5;
                armXMin = -0.5;
            }

            double armXPower = Range.clip(gamepad2.right_stick_x, armXMin, armXMax);
            armX.setPower(armXPower);

            // Controls arm vertical axis
            // Increments vertical position each dpad input
            if (cur2.dpad_up && !prev2.dpad_up && (armYSetting < 3)) {
                armYSetting++;
                armY.setTargetPosition(armYPositions[armYSetting]);
                armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armY.setPower(1);
            } else if (cur2.dpad_down && !prev2.dpad_down && (armYSetting > 0)) {
                armYSetting--;
                armY.setTargetPosition(armYPositions[armYSetting]);
                armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armY.setPower(1);
            }

            if (!armY.isBusy()) {
                armY.setPower(0);
            }

            // Toggles intake
            if (cur2.right_bumper && !prev2.right_bumper) {
                if (!intaking) {
                    intake.setPower(-1);
                    intaking = true;
                    outtaking = false;
                }
                else {
                    intake.setPower(0);
                    intaking = false;
                    outtaking = false;
                }
            } else if (cur2.left_bumper && !prev2.left_bumper) {
                if (!outtaking) {
                    intake.setPower(1);
                    intaking = false;
                    outtaking = true;
                }
                else {
                    intake.setPower(0);
                    intaking = false;
                    outtaking = false;
                }
            }

            // Toggles intake hatch
            if (!hatchOpen && (cur2.right_trigger > 0.1) && (prev2.right_trigger < 0.1)) {
                hatch.setPosition(-1);
                hatchOpen = true;
            } else if (hatchOpen && (cur2.right_trigger > 0.1) && (prev2.right_trigger < 0.1)) {
                hatch.setPosition(1);
                hatchOpen = false;
            }

            // Toggles spinner
            if (cur2.a && !prev2.a) {
                if (!isSpinningForth) {
                    spinner.setPower(1);
                    isSpinningForth = true;
                    isSpinningBack = false;
                }
                else {
                    spinner.setPower(0);
                    isSpinningForth = false;
                    isSpinningBack = false;
                }
            } else if (cur2.x && !prev2.x) {
                if (!isSpinningBack) {
                    spinner.setPower(-1);
                    isSpinningForth = false;
                    isSpinningBack = true;
                }
                else {
                    spinner.setPower(0);
                    isSpinningForth = false;
                    isSpinningBack = false;
                }
            }

            // Reset Field Centric button
            if (cur1.a && !prev1.a) {
                drive.newOffset();
            }

            // Updates prev1 & 2
            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Send FTC Dashboard Packets
            packet.put("loop count", loops);
            packet.put("fr power", fr.getPower());
            packet.put("fl power", fl.getPower());
            packet.put("rr power", rr.getPower());
            packet.put("rl power", rl.getPower());
            packet.put("armXPower", armXPower);
            packet.put("armYStick", -gamepad2.right_stick_y);
            packet.put("armXStick", -gamepad2.left_stick_x);
            packet.put("armXEnc", armX.getCurrentPosition());
            packet.put("armYEnc", armY.getCurrentPosition());
            packet.put("armYSetting", armYSetting);
            packet.put("intakeControl", cur2.right_bumper);
            packet.put("intakePower", intake.getPower());
            dashboard.sendTelemetryPacket(packet);

            loops++;
        }
    }
}
