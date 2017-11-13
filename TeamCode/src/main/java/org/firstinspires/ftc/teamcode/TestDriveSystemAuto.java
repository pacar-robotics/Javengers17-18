package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.rr_Constants.ROBOT_TRACK_DISTANCE;

@TeleOp(name = "Test Auto Drive", group = "Test")

public class TestDriveSystemAuto extends rr_OpMode {

    rr_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeIsActive()) {
            turnUsingEncoders(this, 50, 90, rr_Constants.TurnDirectionEnum.Clockwise);
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot();
    }

    /**
     * Using encoders, this method turns the Robot clockwise or counter clockwise based on angle given.
     * Calculates the turn distance by multiplying the angle by conversion factors to get to an encoder value
     *
     * @param aOpMode       an object of the vv_OpMode class
     * @param power         power in which to apply to each motor
     * @param angle         angle in which the robot will turn to based on the current position as 0 degree
     * @param TurnDirection Turns either Clockwise or Counterclockwise
     * @throws InterruptedException
     */
    private void turnUsingEncoders(rr_OpMode aOpMode, float power, float angle, rr_Constants.TurnDirectionEnum TurnDirection)
            throws InterruptedException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.

        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECANUM_WHEEL_DIAMETER * 360));


        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance, true);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance, true);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }
}
