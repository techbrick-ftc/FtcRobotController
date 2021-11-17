package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="RedAuto", group="teleop")
public class RedAuto extends AutoImport {

    public RedAuto() { super(-62, 29, 225, 150, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            slauto.drive(55, -60, -90, 0.75, this);
            doSpinny(true, 3000);

            stopCamera();
        }
    }
}
