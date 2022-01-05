package org.firstinspires.ftc.teamcode.mains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libs.Nikolaj;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;

import static java.lang.Math.PI;

@Autonomous(name="Red Auto Park",group="Auto")
public class RedAutoPark extends LinearOpMode implements TeleAuto {
    // Pre-init
    private final Nikolaj robot = new Nikolaj();


    @Override
    public void runOpMode() {
        // Init
        robot.setup(hardwareMap);

        robot.flMotor().setDirection(DcMotor.Direction.REVERSE);
        robot.rlMotor().setDirection(DcMotor.Direction.REVERSE);

        robot.flMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rlMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rrMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            robot.getLifter().setPower(-1);
            sleep(1000);
            robot.getLifter().setPower(0);

            strafe(0.5, 1200);
            forwards(0.5, 700);
            sleep(100);
            strafe(0.5, 1500);
            sleep(500);

            robot.getLifter().setPower(1);
            sleep(1000);
            robot.getLifter().setPower(0);
        }
    }

    private void forwards(double power, int time) {
        robot.flMotor().setPower(-power);
        robot.frMotor().setPower(-power);
        robot.rlMotor().setPower(-power);
        robot.rrMotor().setPower(-power);

        sleep(time);

        halt();
    }

    private void strafe(double power, int time) {
        robot.flMotor().setPower(-power);
        robot.frMotor().setPower(power);
        robot.rlMotor().setPower(power);
        robot.rrMotor().setPower(-power);

        sleep(time);

        halt();
    }

    private void halt() {
        robot.flMotor().setPower(0);
        robot.frMotor().setPower(0);
        robot.rlMotor().setPower(0);
        robot.rrMotor().setPower(0);
    }

}