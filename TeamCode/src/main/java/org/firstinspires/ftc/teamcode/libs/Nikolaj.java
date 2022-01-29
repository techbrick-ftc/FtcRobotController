package org.firstinspires.ftc.teamcode.libs;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
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
    private DcMotor arm;

    private CRServo lSrv;
    private CRServo rSrv;

    private TouchSensor touch;

    public void setup(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rlMotor = hardwareMap.get(DcMotor.class, "leftRear");
        rrMotor = hardwareMap.get(DcMotor.class, "rightRear");

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lifter.setMode(STOP_AND_RESET_ENCODER);
        lifter.setMode(RUN_USING_ENCODER);
        lifter.setDirection(REVERSE);
        lifter.setZeroPowerBehavior(BRAKE);
        arm.setTargetPosition(0);
        arm.setMode(RUN_TO_POSITION);
        arm.setZeroPowerBehavior(BRAKE);
        arm.setPower(1);


        lSrv = hardwareMap.get(CRServo.class, "servo1");
        rSrv = hardwareMap.get(CRServo.class, "servo0");

        flMotor.setDirection(REVERSE);
        rrMotor.setDirection(REVERSE);

        touch = hardwareMap.get(TouchSensor.class, "touch");
    }

    public DcMotor flMotor() { return this.flMotor; }
    public DcMotor frMotor() { return this.frMotor; }
    public DcMotor rlMotor() { return this.rlMotor; }
    public DcMotor rrMotor() { return this.rrMotor; }

    public DcMotor getLifter() { return lifter; }
    public DcMotor getArm() { return arm; }

    public CRServo getLSrv() { return lSrv; }
    public CRServo getRSrv() { return rSrv; }

    public TouchSensor getTouch() { return touch; }
}
