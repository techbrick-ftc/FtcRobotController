package org.firstinspires.ftc.teamcode.tests;
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

@TeleOp(name="EncoderTest", group="teleop")
public class EncoderTest extends AutoImport{
    public EncoderTest() { super(31, -56, 225, 150, 255, 150); }

    FieldCentric drive = new FieldCentric();

    public boolean driverAbort() {
        return gamepad1.y;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();
        
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();
        
        waitForStart();
        
        while (opModeIsActive()) {

            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            if(gamepad1.a){
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }

            if (gamepad1.dpad_left){
                armX.setPower(1);
            }
            else if (gamepad1.dpad_right){
                armX.setPower(-1);
            }
            else {
                armX.setPower(0);
            }

            if (cur1.dpad_up && !prev1.dpad_up){
                armY.setTargetPosition(3000);
                armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armY.setPower(.5);
            }

            else if (cur1.dpad_down && !prev1.dpad_down) {
                armY.setTargetPosition(2000);
                armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armY.setPower(.5);
            }

            packet.put("ArmX", armX.getCurrentPosition());
            packet.put("ArmY", armX.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);

            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }
        }
    }
}