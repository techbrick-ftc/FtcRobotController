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
            drive.setUp(motors, motorAngles, imu);
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
        boolean isSpinningForth = false;
        boolean isSpinningBack = false;
        boolean hatchOpen = false;
        double armXMin = -0.5;
        double armXMax = 0.5;

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

            // Controls arm
            // Enforces encoder barriers at 90 and -90 degrees of starting position
            /*if (armX.getCurrentPosition() >= 356) { // constant is 1/4 of full encoder rotation
                armXMax = 0;
            } else if (armX.getCurrentPosition() <= -356) {
                armXMin = 0;
            } else {
                armXMax = 0.5;
                armXMin = -0.5;
            }

            double armYPower = Range.clip(-gamepad2.left_stick_y, -0.5, 0.5);
            armY.setPower(armYPower);
            double armXPower = Range.clip(-gamepad2.left_stick_x, armXMin, armXMax);
            armX.setPower(armXPower);*/

            // Toggles spinner
            if (!isSpinningForth && cur2.a && !prev2.a) {
                spinner.setPower(1);
                isSpinningForth = true;
            } else if (!isSpinningBack && cur2.x && !prev2.x) {
                spinner.setPower(-1);
                isSpinningBack = true;
            } else if ((isSpinningForth || isSpinningBack) && (cur2.a && !prev2.a) || (cur2.x && !prev2.x)) {
                spinner.setPower(0);
                isSpinningForth = false;
                isSpinningBack = false;
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
            dashboard.sendTelemetryPacket(packet);

            loops++;
        }
    }
}
