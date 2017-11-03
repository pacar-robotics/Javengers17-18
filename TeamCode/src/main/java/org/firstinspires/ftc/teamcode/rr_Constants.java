package org.firstinspires.ftc.teamcode;

/**
 * Class rr_Constants provides all constants that will be used
 * Ensures that all values are in sync
 * Also contains enumerations for direction, motors and beacon buttons
 */
public class rr_Constants {

    //Constants for Debugging
    final static boolean DEBUG = false;
    final static long DEBUG_MESSAGE_DISPLAY_TIME = 10; //time to flash DEBUG message on telemtry
    final static boolean DEBUG_AUTO_CLEAR = false;  //will the screen refresh after each telemtry

    //Constants for Diagnostics
    final static int MAX_ROBOT_DIAGNOSTIC_TESTS = 20;
    final static String DIAG_RESULTS_RELATIVE_FILE_PATH = "/PACAR/DiagResults.xml";

    // Encoder constants
    final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
    final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;
    final static float MATRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1478.4f;

    //Motor constants
    final static float MOTOR_LOWER_POWER_THRESHOLD = 0.20f;
    final static int MAX_MOTOR_LOOP_TIME = 10000;     //max time to wait in a tight loop
    final static int ENCODED_MOTOR_STALL_TIME_DELTA = 200; //time to wait in stall check code

    //encoder clicks to check motors for stall
    final static int ENCODED_MOTOR_STALL_CLICKS_ANDYMARK = 10;
    final static int ENCODED_MOTOR_STALL_CLICKS_TETRIX = 10;
    final static int ENCODED_MOTOR_STALL_CLICKS_MATRIX = 5;


    //Base Motor power limits
    final static float MOTOR_RAMP_FB_POWER_LOWER_LIMIT = 0.3f;
    final static float MOTOR_RAMP_FB_POWER_UPPER_LIMIT = 0.78f;
    final static float MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT = 0.6f;
    final static float MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT = 0.78f;
    final static float MAX_ROBOT_TURN_MOTOR_VELOCITY = 0.78f;
    final static float MIN_ROBOT_TURN_MOTOR_VELOCITY = 0.15f;

    //Base Motor Trims
    final static float LEFT_MOTOR_TRIM_FACTOR = 0.95f;
    final static float RIGHT_MOTOR_TRIM_FACTOR = 1.0f;

    //Cube Arm Constants TODO: CHANGE
    final static int CUBE_ARM_MAX_DURATION = 5000;
    final static float CUBE_ARM_UPPER_LIMIT = 0;
    final static float CUBE_ARM_LOWER_LIMIT = 10000;
    final static int ONE_CUBE_ROW_4 = 0; //Highest
    final static int ONE_CUBE_ROW_3 = 0;
    final static int ONE_CUBE_ROW_2 = 0;
    final static int ONE_CUBE_ROW_1 = 0; //Lowest
    final static int TWO_CUBE_POS_1 = 0; //Lowest
    final static int TWO_CUBE_POS_2 = 0;
    final static int TWO_CUBE_POS_3 = 0; //Highest
    final static float CUBE_ARM_GRAB = 0;
    final static float CUBE_ARM_SCORING_POWER = .5f;
    final static float CUBE_ARM_POWER_FACTOR = 0.20f;
    final static float CUBE_CLAW_OPEN = 0.72f;
    final static float CUBE_CLAW_ONE_CLOSED = 0.445f;
    final static float CUBE_CLAW_TWO_CLOSED = 0.1f;
    final static float CUBE_ORIENTATION_HORIZONTAL = 0.9f;
    final static float CUBE_ORIENTATION_VERTICAL = 0.44f;

    //Relic Arm Constants TODO: Change
    final static float RELIC_EXTEND_POWER = 1.0f;
    final static float RELIC_RETRACT_POWER = 1.0f;
    final static int RELIC_UPPER_LIMIT = 1000;
    final static int RELIC_LOWER_LIMIT = 1000;
    final static int RELIC_ZONE_1 = 1000;
    final static int RELIC_ZONE_2 = 1000;
    final static int RELIC_ZONE_3 = 1000;
    final static int RELIC_MAX_DURATION = 5000;
    final static float RELIC_CLAW_ANGLE_GRAB = 0.1f;
    final static float RELIC_CLAW_ANGLE_EXTEND = 0.1f;
    final static float RELIC_CLAW_OPEN = 0.1f;
    final static float RELIC_CLAW_CLOSED = 0.1f;
    final static double RELIC_ARM_RETRACT_POWER = 0;
    final static float RELIC_ARM_EXTEND_POWER_FACTOR = .8f;
    final static float RELIC_ARM_RETRACT_POWER_FACTOR = .8f;
    final static float RELIC_CLAW_ANGLE_MAX = .1f;
    final static float RELIC_CLAW_ANGLE_MIN = .1f;

    //Jewel Arm Constants TODO: CHANGE
    final static float JEWEL_KNOCKER_LEFT = 0.1f;
    final static float JEWEL_KNOCKER_RIGHT = 0.1f;
    final static float JEWEL_KNOCKER_NEUTRAL = 0.1f;
    final static float JEWEL_ARM_IN = 0.1f;
    final static float JEWEL_ARM_OUT = 0.1f;

    //index of motors
    final static int FRONT_LEFT_MOTOR = 0;
    final static int FRONT_RIGHT_MOTOR = 1;
    final static int BACK_LEFT_MOTOR = 2;
    final static int BACK_RIGHT_MOTOR = 3;
    final static int CUBE_ARM = 4;
    final static int RELIC_ARM_EXTEND = 5;

    //Mecanum wheel properties
    final static float MECANUM_WHEEL_DIAMETER = 4f;   //in inches
    final static float MECANUM_WHEEL_ENCODER_MARGIN = 50;
    final static float MECANUM_WHEEL_SIDE_TRACK_DISTANCE = 13.0f;
    final static float MECANUM_WHEEL_FRONT_TRACK_DISTANCE = 14.5f;

    //Power Factors
    final static float STANDARD_DRIVE_POWER_FACTOR = 0.7f;
    final static float TURN_POWER_FACTOR = 0.95f;
    final static float SCORING_DRIVE_POWER_FACTOR = 0.25f;  //TODO: CHANGE LATER

    //Gamepad Thresholds
    final static float ANALOG_STICK_THRESHOLD = .25f;
    final static float TRIGGER_THRESHOLD = .10f;

    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 13.7f; //adjusted from observation.

    //Sensor Constants TODO: CHANGE
    final static float JEWEL_RED_THRESHOLD = 0.5f;
    final static float JEWEL_BLUE_THRESHOLD = 0.5f;

    //the value below for floor color sensor is the fallback value.
    //it needs to be adjusted if the height of the floor color sensor
    //is changed.
    //normally Floor sensor calibration automatically sets values in
    //an XML file adjusting for the light in the competition venue.

    final static float FLOOR_WHITE_THRESHOLD = 30f; //may need to calibrate
    final static float FLOOR_WHITE_MARGIN = 3f; //may need to calibrate

    //Range Sensor Threshold
    final static double RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD = 1.0d;

    //Timers
    public static final int GENERIC_TIMER = 0;
    public static final int DPAD_TIMER = 1;

    //Enumerations
    DirectionEnum Direction;
    TurnDirectionEnum TurnDirection;
    JewelColorEnum JewelColor;


    enum AllianceColorEnum {
        BLUE,
        RED
    }

    enum JewelColorEnum {
        BLUE,
        RED,
        UNKNOWN
    }

    //values that control the worm drive motor to adjust tension of the Launch arm

    // Direction of movement for autonomous
    enum DirectionEnum {
        Forward, Backward, SidewaysLeft, SidewaysRight
    }

    // Direction of turning
    enum TurnDirectionEnum {
        Clockwise, Counterclockwise
    }
}
