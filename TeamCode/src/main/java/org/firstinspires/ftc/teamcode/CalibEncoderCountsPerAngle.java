package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.rr_Constants.*;

/**
 * Created by Kenneth on 12/29/2017.
 */

@Autonomous(name = "Calibrate Encoder Turning", group = "Calibration")
public class CalibEncoderCountsPerAngle extends rr_OpMode{

    rr_AutoLib rrAutoLib; //initializes bosch gyro heading to 0


    @Override
    public void runOpMode() throws InterruptedException {
        int encoderCountPerDegree;

        //2240 encoder counts is almost 2 rotations of the motor
        rrAutoLib.robot.runRobotToPosition(this, .4f, .4f, .4f, .4f, 2240, -2240, 2240, -2240, true );

        //divides encoder counts by the current gyro heading, which is overal
        encoderCountPerDegree = 2240 / (int) rrAutoLib.robot.getBoschGyroSensorHeading(this);

        DBG("Encoder Count Per Angle = " + encoderCountPerDegree);
    }
}
