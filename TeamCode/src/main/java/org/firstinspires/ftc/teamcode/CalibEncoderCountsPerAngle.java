package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.rr_Constants.*;

/**
 * Created by Kenneth on 12/29/2017.
 */

@Autonomous(name = "Calibrate Encoder Turn Angle", group = "Calibration")
public class CalibEncoderCountsPerAngle extends rr_OpMode{

    rr_AutoLib rrAutoLib; //initializes bosch gyro heading to 0


    @Override
    public void runOpMode() throws InterruptedException {
        final int trialCount=10;
        float encoderCountPerDegreeSum=0;
        float encoderCountPerDegreeFinal=0;
        final int encoderCountToBeTurned=1500;
        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);


        for(int i=0;i<trialCount;i++) {
            //trials
            rrAutoLib.robot.initIMUGyro(this); //initialize gyro. to reset angle.

            rrAutoLib.robot.runRobotToPosition(this, .4f, .4f, .4f, .4f,
                    encoderCountToBeTurned, -encoderCountToBeTurned,
                    encoderCountToBeTurned, -encoderCountToBeTurned, true);

            //divides encoder counts by the current gyro heading, which would have tracked the turn in degrees
            encoderCountPerDegreeSum +=
                    ((float) encoderCountToBeTurned)/rrAutoLib.robot.getBoschGyroSensorHeading(this);
            encoderCountPerDegreeFinal=encoderCountPerDegreeSum/(i+1); //+1 for zero based loop index
            telemetryAddLine("Degrees Turned:"+rrAutoLib.robot.getBoschGyroSensorHeading(this));
            telemetryAddLine("Encoder Fl:"+rrAutoLib.robot.getMotorPosition(this, FRONT_LEFT_MOTOR)
                    + "FR:"+ rrAutoLib.robot.getMotorPosition(this, FRONT_RIGHT_MOTOR)
                    + "BL:"+ rrAutoLib.robot.getMotorPosition(this, BACK_LEFT_MOTOR)
                    + "BR:"+ rrAutoLib.robot.getMotorPosition(this, BACK_RIGHT_MOTOR));
            telemetryAddLine("Encoder Count Per Degrees Turned Average:"+encoderCountPerDegreeFinal);

            telemetryUpdate();
            Thread.sleep(2000);

        }
        //report average of all trials.

        telemetryAddLine("Encoder Count Per Degrees Turned Average Final:"+encoderCountPerDegreeFinal);
        telemetryAddLine("This value should be plugged into the turnByEncoder method in robot");

        telemetryUpdate();

        Thread.sleep(10000);
    }
}
