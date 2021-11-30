package org.firstinspires.ftc.teamcode.libs;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Nikolaj {
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor rlMotor;
    private DcMotor rrMotor;

    private DcMotor lifter;

    private CRServo intake;
    private CRServo spinner;

    private TouchSensor touch;

    public void setup(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rlMotor = hardwareMap.get(DcMotor.class, "leftRear");
        rrMotor = hardwareMap.get(DcMotor.class, "rightRear");

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        lifter.setMode(STOP_AND_RESET_ENCODER);
        lifter.setMode(RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(BRAKE);

        intake = hardwareMap.get(CRServo.class, "servo0");
        spinner = hardwareMap.get(CRServo.class, "servo1");

        intake.setDirection(REVERSE);
        spinner.setDirection(REVERSE);

        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    public DcMotor flMotor() { return this.flMotor; }
    public DcMotor frMotor() { return this.frMotor; }
    public DcMotor rlMotor() { return this.rlMotor; }
    public DcMotor rrMotor() { return this.rrMotor; }

    public DcMotor getLifter() { return lifter; }

    public CRServo getIntake() { return intake; }
    public CRServo getSpinner() { return spinner; }

    public TouchSensor getTouch() { return touch; }
}
