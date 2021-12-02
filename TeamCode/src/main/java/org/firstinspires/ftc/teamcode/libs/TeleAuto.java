package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface TeleAuto {
    boolean opModeIsActive();
    boolean driverAbort();
    void sleep(long milliseconds);
    void idle();
}
