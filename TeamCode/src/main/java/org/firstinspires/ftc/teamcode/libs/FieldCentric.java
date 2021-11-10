// This is the class that allows us to use field centric in teleop.

package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

public class FieldCentric {
    // Variable setup, all will be explained within code
    private DcMotor[] motors;
    private double[] wheelAngles;
    private double r;
    private double theta;
    private double rotation = 0;
    private double currentAngle;
    private double offsetAngle = 0;

    public void setUp(DcMotor[] motors, double[] wheelAngles) throws Exception {
        // Check if we have angles for every motor, and vice versa
        if (motors.length != wheelAngles.length) {
            throw new java.lang.Exception("Motor and wheelAngle arrays do not have same length.\nCheck your code!!!");
        }

        this.motors = motors;
        this.wheelAngles = wheelAngles;

        getAngle();
        this.rotation = currentAngle;

        for (double wheelAngle : wheelAngles) {
            wheelAngle -= currentAngle;
        }

        newOffset();
    }

    private void getAngle() {
        currentAngle = wrap(getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offsetAngle);
    }

    /**
     * Run every loop to drive robot using field centricity
     * @param x The input to control robot's x movement
     * @param y The input to control robot's y movement
     * @param turn The input to control robot's turn
     */
    public void Drive(double x, double y, double turn) {
        /*
            Get the current angle
         */
        getAngle();

        /*
            Set r (for polar coords) to the distance of the point (x,y) from (0,0)
            (The speed)
         */
        r = Math.sqrt(x*x+y*y);

        /*
            Set theta (for polar coords) to the theta of the point (x,y) to the x-axis at the origin
            (The direction to go)
         */
        theta = Math.atan2(x, y);

        /*
            Add current angle to account for rotation (since we are getting the theta from a controller
            axis (-1.0 to 1.0) we don't know the angle we are currently at
         */
        double newTheta = theta + currentAngle;

        /*
            Sets rotation to current angle, while driver is intentionally turning the robot. This makes
            it so the robot does not turn from things like poor weight distribution.
        */

        /*if (turn != 0) {
            rotation = currentAngle + turn;
        }

        if (rotation > PI) {
            rotation = PI - .01;
        } else if (rotation > -PI) {
            rotation = -PI + .01;
        }

        double newRotation = rotation - currentAngle;*/

        /*
            Get the angle of the wheel and subtract the newTheta (because newTheta is clockwise and
            math is counter-clockwise)
         */
        motors[0].setPower(-(Math.sin(wheelAngles[0] - newTheta) * r + (turn / 1.2)));
        motors[1].setPower(Math.sin(wheelAngles[1] - newTheta) * r + (turn / 1.2));
        motors[2].setPower(Math.sin(wheelAngles[2] - newTheta) * r + (turn / 1.2));
        motors[3].setPower(-(Math.sin(wheelAngles[3] - newTheta) * r + (turn / 1.2)));
    }

    public void newOffset() {
        offsetAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - Math.PI/2;
    }

    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }
}