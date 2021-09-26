package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="DetectionTest", group="tests")
public class DetectionTest extends AutoImport {

    public DetectionTest() {
        // change cam values
        super(0, 0, 35, 172, 135, 172);
    }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            sleep(30000);
        }
    }
}