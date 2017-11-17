package org.firstinspires.ftc.teamcode;

/**
 * Class rr_Constants provides all constants that will be used
 * Ensures that all values are in sync
 * Also contains enumerations for direction, motors and beacon buttons
 */
public class rr_Constants {

    //Constants for Debugging
    final static boolean DEBUG = true;
    final static int DEBUG_LEVEL = 0;
    final static long DEBUG_MESSAGE_DISPLAY_TIME = 10; //time to flash DEBUG message on telemetry
    // setting DEBUG_AUTO_CLEAR to false interferes with Vuforia code
    final static boolean DEBUG_AUTO_CLEAR = true;  //will the screen refresh after each telemetry

    // Glyph scoring adjustment constants
    final static float BLUE_ONE_SCORING_DISTANCE = 35;
    final static float BLUE_TWO_SCORING_DISTANCE = 18;
    final static float RED_ONE_SCORING_DISTANCE = 32;
    final static float RED_TWO_SCORING_DISTANCE = 10;

    //Constants for Diagnostics
    final static int MAX_ROBOT_DIAGNOSTIC_TESTS = 20;
    final static String DIAG_RESULTS_RELATIVE_FILE_PATH = "/PACAR/DiagResults.xml";

    // Encoder constants
    final static int TETRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1440;
    final static int ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1120;
    final static float MATRIX_MOTOR_ENCODER_COUNTS_PER_REVOLUTION = 1478.4f;
    final static int MOTOR_ENCODER_THRESHOLD = 50;

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
    final static float CUBE_ARM_UPPER_LIMIT = -2216;
    final static float CUBE_ARM_LOWER_LIMIT = 0;
    final static int ONE_CUBE_ROW_4 = 0; //Highest
    final static int ONE_CUBE_ROW_3 = 0;
    final static int ONE_CUBE_ROW_2 = 0;
    final static int ONE_CUBE_ROW_1 = 0; //Lowest
    final static int TWO_CUBE_POS_1 = 0; //Lowest
    final static int TWO_CUBE_POS_2 = 0;
    final static int TWO_CUBE_POS_3 = 0; //Highest
    final static int CUBE_ARM_GRAB = 0;
    final static int CUBE_ARM_MIDDLE = -500;
    final static int CUBE_ARM_TOP = -1500;
    final static int CUBE_ARM_SAFE_POS = -700;
    final static float CUBE_ARM_SCORING_POWER = .5f;
    final static float CUBE_ARM_POWER_FACTOR = 0.20f;
    final static float CUBE_ARM_RAISE_POWER = -0.4f;
    final static float CUBE_ARM_LOWER_POWER = 0.1f;
    final static float CUBE_CLAW_OPEN = 0.625f;
    final static float CUBE_CLAW_ONE_CLOSED = 0.35f;
    final static float CUBE_CLAW_ONE_RELEASE = 0.2f;
    final static float CUBE_CLAW_TWO_CLOSED = 0.77f;
    final static float CUBE_ORIENTATION_HORIZONTAL = 0.15f;
    final static float CUBE_ORIENTATION_VERTICAL = 0.44f;

    //Relic Arm Constants
    final static float RELIC_WINCH_EXTEND_POWER_FACTOR = .8f;   // TODO: Change
    final static float RELIC_WINCH_RETRACT_POWER_FACTOR = .8f;  // TODO: Change
    final static int RELIC_WINCH_REST = -220;                   // TODO: Change
    final static int RELIC_WINCH_UPPER_LIMIT = 11000;           // TODO: Change
    final static float RELIC_ARM_GRAB = 0.3f;
    final static float RELIC_ARM_EXTEND_UP = 0.75f;//feet of robot pointing up
    final static float RELIC_ARM_EXTEND_IN = 0.0f; //feet of relic toward the robot
    final static float RELIC_ARM_MAX = 1f;              // TODO: Change
    final static float RELIC_ARM_MIN = .1f;              // TODO: Change
    final static float RELIC_CLAW_OPEN = 0.8f;
    final static float RELIC_CLAW_OPEN_STABILIZED=0.4f;
    final static float RELIC_CLAW_CLOSED = 0.21f;


    //Jewel Arm Constants TODO: CHANGE
    final static float JEWEL_PUSHER_LEFT = 0.63f;
    final static float JEWEL_PUSHER_RIGHT = 0.28f;
    final static float JEWEL_PUSHER_NEUTRAL = 0.48f;
    final static float JEWEL_ARM_UP = 0.55f;
    final static float JEWEL_ARM_DOWN_READ = .95f;
    final static float JEWEL_ARM_DOWN_PUSH = .98f;


    //index of motors
    final static int FRONT_LEFT_MOTOR = 0;
    final static int FRONT_RIGHT_MOTOR = 1;
    final static int BACK_LEFT_MOTOR = 2;
    final static int BACK_RIGHT_MOTOR = 3;
    final static int CUBE_ARM = 4;
    final static int RELIC_WINCH = 5;

    //Mecanum wheel properties
    final static float MECANUM_WHEEL_DIAMETER = 4f;   //in inches
    final static float MECANUM_WHEEL_ENCODER_MARGIN = 50;
    final static float MECANUM_WHEEL_SIDE_TRACK_DISTANCE = 13.0f;
    final static float MECANUM_WHEEL_FRONT_TRACK_DISTANCE = 14.5f;

    //Power Factors
    final static float STANDARD_DRIVE_POWER_FACTOR = 0.8f;
    final static float TURN_POWER_FACTOR = 0.5f;
    final static float SCORING_DRIVE_POWER_FACTOR = 0.25f;  //TODO: CHANGE LATER

    //Gamepad Thresholds
    final static float ANALOG_STICK_THRESHOLD = .25f;
    final static float TRIGGER_THRESHOLD = .10f;

    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 12.25f; //adjusted from observation.

    //Sensor Constants TODO: CHANGE
    final static int JEWEL_COLOR_MARGIN = 10;

    //TODO: Change these
    final static float FLOOR_RED_THRESHOLD = 0.5f;
    final static float FLOOR_BLUE_THRESHOLD = 0.5f;

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
