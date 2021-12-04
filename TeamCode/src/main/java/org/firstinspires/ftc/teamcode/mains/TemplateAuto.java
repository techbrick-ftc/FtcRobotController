package org.firstinspires.ftc.teamcode.mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.CameraAuto;
import org.firstinspires.ftc.teamcode.libs.Globals;
import org.firstinspires.ftc.teamcode.libs.Nikolaj;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;

import static java.lang.Math.PI;

@Autonomous(name="",group="")
public class TemplateAuto extends LinearOpMode implements TeleAuto {
    // Pre-init
    private final CameraAuto auto = new CameraAuto();
    private final Nikolaj robot = new Nikolaj();

    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);
        auto.setUp(
                new DcMotor[]{robot.frMotor(), robot.rrMotor(), robot.rlMotor(), robot.flMotor()},
                new double[]{PI/4, 3*PI/4, 5*PI/4, 7*PI/4},
                null,
                AxesReference.EXTRINSIC,
                hardwareMap
        );
    
        waitForStart();
    
        // Pre-run
    
        if (opModeIsActive()) {
            // Autonomous instructions
            while (Globals.getCamera().getLastReceivedCameraUpdate().confidence == T265Camera.PoseConfidence.Failed) { idle(); }
            auto.goToPosition(10, 0, this);
            auto.goToRotation(PI, this);
            auto.goTo(0, -10, PI/4, this);
        }
    }
}
