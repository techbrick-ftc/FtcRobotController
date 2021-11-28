package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="RedAuto", group="teleop")
public class RedAuto extends AutoImport {

    public RedAuto() { super(65, -40, 225, 150, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to spinner and does spinny
            slauto.drive(55, -60, -90, 0.75, this);
            doSpinny(true, 1000);

            // Starts lifting arm
            armY.setTargetPosition(-2000);
            armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armY.setPower(1);

            sleep(1000);

            // Goes to the other side of the wall in 3 connected movements
            slauto.drive(50, -40, 0, 0.8, 0, this, false, true);
            slauto.drive(65, 0, 0, 0.5, 0, this, false, false);
            slauto.drive(65, 50, 0, 0.5, 0, this, true, false);

            stopCamera();
        }
    }
}
