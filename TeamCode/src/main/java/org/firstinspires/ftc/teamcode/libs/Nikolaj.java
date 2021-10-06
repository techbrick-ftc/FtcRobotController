package org.firstinspires.ftc.teamcode.libs;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Nikolaj {
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor rlMotor;
    private DcMotor rrMotor;

    public void setup(HardwareMap hardwareMap) {
        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        rlMotor = hardwareMap.get(DcMotor.class, "rl");
        rrMotor = hardwareMap.get(DcMotor.class, "rr");

        flMotor.setDirection(REVERSE);
        rrMotor.setDirection(REVERSE);
    }

    public DcMotor flMotor() { return this.flMotor; }
    public DcMotor frMotor() { return this.frMotor; }
    public DcMotor rlMotor() { return this.rlMotor; }
    public DcMotor rrMotor() { return this.rrMotor; }
}
