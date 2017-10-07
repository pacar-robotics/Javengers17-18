package org.firstinspires.ftc.teamcode;

/**
 * Created by Sarah Chacko, Krittika Negandhi, and Rahul Ramkumar on 9/25/2016.
 */

/**
 * Class vv_Constants provides all constants that will be used
 * Ensures that all values are in sync
 * Also contains enumerations for direction and motors
 */
public class rrConstants
{
    // Encoder constants
    final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
    final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;
    final static float MATRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1478.4f;

    final static int ARM_MOTOR_ENCODER_COUNTS_PER_REVOLUTION =
            ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION * 3 / 2;  //Gear ratio for Choo-Choo

    final static float MOTOR_LOWER_POWER_THRESHOLD = 0.20f;

    //forward/backward power limits
    final static float MOTOR_RAMP_FB_POWER_LOWER_LIMIT = 0.3f;
    final static float MOTOR_RAMP_FB_POWER_UPPER_LIMIT = 0.78f;

    //sideways power limit
    final static float MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT = 0.6f;
    final static float MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT = 0.78f;

    //sideways ultrasonic travel power limit
    final static float MOTOR_ULTRASONIC_SIDEWAYS_POWER_LOWER_LIMIT = 0.18f;
    final static float MOTOR_ULTRASONIC_SIDEWAYS_POWER_UPPER_LIMIT = 0.78f;


    final static int GYRO_ERROR = 8;

    // Mecanum wheel properties
    final static float MECCANUM_WHEEL_DIAMETER = 4f;   //in inches
    final static float MECCANUM_WHEEL_ENCODER_MARGIN = 50;
    final static float MECCANUM_WHEEL_SIDE_TRACK_DISTANCE = 13.0f;
    final static float MECCANUM_WHEEL_FRONT_TRACK_DISTANCE = 14.5f;

    final static float STANDARD_DRIVE_POWER_FACTOR = 0.7f;

    final static float ANALOG_STICK_THRESHOLD = .25f;
    final static float TRIGGER_THRESHOLD = .10f;

    static final int ROBOT_ANGLE_THRESHOLD = 3;

    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 13.7f; //adjusted from observation.
    //max time to wait in a tight loop, for example in robot turns or autonomous moves
    final static int MAX_MOTOR_LOOP_TIME = 10000;
    //max turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MAX_ROBOT_TURN_MOTOR_VELOCITY = 0.78f;
    //min turn velocity for a motor, to be used in autonomous turns of the robot
    final static float MIN_ROBOT_TURN_MOTOR_VELOCITY = 0.15f;
    //gyro offset to address inertia and gyro lag
    final static int GYRO_OFFSET = 10;

    final static float TURN_POWER = 0.95f;

    final static boolean DEBUG = false;
    //time to flash DEBUG message on telemtry
    final static long DEBUG_MESSAGE_DISPLAY_TIME = 10;

    //controls whether debug messages remain on screen while new ones are written
    final static boolean DEBUG_AUTO_CLEAR = false;

    //time to wait in stall check code
    final static int ENCODED_MOTOR_STALL_TIME_DELTA = 200;

    //encoder clicks to check for Andymark motors for stall
    final static int ENCODED_MOTOR_STALL_CLICKS_ANDYMARK = 10;
    //encoder clicks to check for Andymark motors for stall
    final static int ENCODED_MOTOR_STALL_CLICKS_TETRIX = 10;
    //encoder clicks to check for Andymark motors for stall
    final static int ENCODED_MOTOR_STALL_CLICKS_MATRIX = 5;

    //index of motors
    final static int FRONT_LEFT_MOTOR = 0;
    final static int FRONT_RIGHT_MOTOR = 1;
    final static int BACK_LEFT_MOTOR = 2;
    final static int BACK_RIGHT_MOTOR = 3;

    final static float LEFT_MOTOR_TRIM_FACTOR = 0.95f;
    final static float RIGHT_MOTOR_TRIM_FACTOR = 1.0f;

    final static double EOPD_PROXIMITY_THRESHOLD = 0.00d;
    final static double RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD = 1.2d;
    final static double RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD = 1.0d;

    final static int CAP_BALL_ENCODER_UPPER_LIMIT = 34500; //for matrix motor adustments.
    //including the gear ratio

    public static final int GENERIC_TIMER = 0;
    public static final int DPAD_TIMER = 1;

    DirectionEnum Direction;
    TurnDirectionEnum TurnDirection;

    // Direction of movement for autonomous
    enum DirectionEnum
    {
        Forward, Backward, SidewaysLeft, SidewaysRight
    }

    enum TurnDirectionEnum
    {
        Clockwise, Counterclockwise
    }
}
