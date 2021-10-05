package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;

@TeleOp(name="BasicTele", group="generic")
public class BasicTele extends AutoImport {

    public BasicTele() { super(31, -56, 225, 150, 255, 150); }
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
        DcMotor[] motors = {m1, m2, m3, m4};
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
        boolean isSpinning = false;
        boolean hatchOpen = false;

        // Starting servo positions
        intakeHatch.setPosition(-1);

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
            double armPower = Range.clip(gamepad2.left_stick_y, -0.5, 0.5);
            arm.setPower(armPower);

            // Controls intake
            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(-1);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // Controls intake hatch
            if (!hatchOpen && cur2.b && !prev2.b) {
                intakeHatch.setPosition(1);
                hatchOpen = true;
            } else if (hatchOpen && cur2.b && !prev2.b) {
                intakeHatch.setPosition(-1);
                hatchOpen = false;
            }

            // Toggles spinner
            if (!isSpinning && cur2.a && !prev2.a) {
                spinner.setPower(1);
                isSpinning = true;
            } else if (isSpinning && cur2.a && !prev2.a) {
                spinner.setPower(0);
                isSpinning = false;
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
            packet.put("fr power", m1.getPower());
            packet.put("fl power", m4.getPower());
            packet.put("rr power", m2.getPower());
            packet.put("rl power", m3.getPower());
            dashboard.sendTelemetryPacket(packet);

            loops++;
        }
    }
}
