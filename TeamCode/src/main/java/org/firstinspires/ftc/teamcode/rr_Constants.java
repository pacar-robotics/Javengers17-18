package org.firstinspires.ftc.teamcode;

/**
 * Class rr_Constants provides all constants that will be used
 * Ensures that all values are in sync
 * Also contains enumerations for direction, motors and beacon buttons
 */
public class rr_Constants {

    //Constants for Debugging
    final static boolean DEBUG = false;
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
    final static float MOTOR_LOWER_POWER_THRESHOLD = 0.15f;
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
    final static float LEFT_MOTOR_TRIM_FACTOR = 1.0f;
    final static float RIGHT_MOTOR_TRIM_FACTOR = 1.0f;

    //intake controls

    final static float INTAKE_POWER_LOW = 0.2f;
    final static float INTAKE_POWER_MEDIUM = 0.5f;
    final static float INTAKE_POWER_HIGH = 1.0f;

    //Relic Arm Constants
    final static float RELIC_WINCH_MAX_DURATION = 5000;
    final static float RELIC_WINCH_EXTEND_POWER_FACTOR = .5f;
    final static float RELIC_WINCH_RETRACT_POWER_FACTOR = -.5f;
    final static int RELIC_WINCH_REST = -220;                   // TODO: Change
    final static int RELIC_WINCH_UPPER_LIMIT = 11000;           // TODO: Change
    final static float RELIC_ARM_GRAB = 0.3f;
    final static float RELIC_ARM_EXTEND_UP = 0.75f;//feet of robot pointing up
    final static float RELIC_ARM_OPEN_PULSE = 0.25f;
    final static float RELIC_ARM_MAX = 1f;              // TODO: Change
    final static float RELIC_ARM_MIN = .1f;              // TODO: Change
    final static float RELIC_CLAW_OPEN = 0.8f;
    final static float RELIC_CLAW_OPEN_STABILIZED=0.4f;
    final static float RELIC_CLAW_CLOSED = 0.21f;

    //Cube Tray Constants
    //these numbers need to be tuned based on a calibration op
    final static int TRAY_HEIGHT_COLLECTION_POSITION=0; //also 1 cube position
    final static int TRAY_HEIGHT_2CUBE_POSITION=625;
    final static int TRAY_HEIGHT_MAX_POSITION=950;
    final static float TRAY_LIFT_POWER_FACTOR=1.0f;
    final static float TRAY_LIFT_POWER=1.0f;

    final static float TRAY_FLIP_HORIZONTAL_POSITION=0.6f;
    final static float TRAY_FLIP_COLLECTION_POSITION=0.72f;
    final static float TRAY_FLIP_SCORING_POSITION=0.0f;


    //Cube Pusher Constants
    final static float CUBE_PUSHER_RESTED_POSITION= 0f;
    final static float CUBE_PUSHER_INIT_POSITION= 0.85f;
    final static float CUBE_PUSHER_PUSHED_POSITION= 1.0f;


    //Jewel Arm Constants
    final static float JEWEL_PUSHER_LEFT = 0.90f; // make bigger
    final static float JEWEL_PUSHER_RIGHT = 0.10f; // make smaller
    final static float JEWEL_PUSHER_NEUTRAL = 0.50f;
    final static float JEWEL_ARM_UP = 0.25f;
    //final static float JEWEL_ARM_DOWN_READ = .89f;
    final static float JEWEL_ARM_DOWN_PUSH = .92f;
    static final double JEWEL_ARM_INCREMENT = 0.01;     // amount to slew servo each cycle
    static final int JEWEL_ARM_CYCLE = 50;     // period of each cycle
    final static float BOTTOM_JEWEL_POSITION = 0.9f;

    //Jewel Sensors
    final static float JEWEL_COLOR_LUMINOSITY_THRESHOLD = 10f;
    final static float JEWEL_COLOR_DIFFERENTIAL_THRESHOLD = 5f;
    final static int JEWEL_COLOR_FILTER_COUNT = 11; //must be odd

    //index of motors
    final static int FRONT_LEFT_MOTOR = 0;
    final static int FRONT_RIGHT_MOTOR = 1;
    final static int BACK_LEFT_MOTOR = 2;
    final static int BACK_RIGHT_MOTOR = 3;
    final static int TRAY_LIFT_MOTOR= 4;
    final static int RELIC_WINCH_MOTOR = 5;
    final static int INTAKE_RIGHT_MOTOR=6;
    final static int INTAKE_LEFT_MOTOR=7;


    //Mecanum wheel properties
    final static float MECANUM_WHEEL_DIAMETER = 4f*1.5f;   //in inches multiplied by gearing factor.
    final static float MECANUM_WHEEL_ENCODER_MARGIN = 50;
    final static float MECANUM_WHEEL_SIDE_TRACK_DISTANCE = 9.5f;
    final static float MECANUM_WHEEL_FRONT_TRACK_DISTANCE = 14.75f;

    //encoder count calculations from CalibEncoderTurn
    final static float ENCODER_COUNT_PER_DEGREE_TURN=13.70f;

    //Power Factors
    final static float STANDARD_DRIVE_POWER_FACTOR = 0.95f;
    final static float FIELD_ORIENTED_DRIVE_POWER_FACTOR = 0.95f;
    final static float TURN_POWER_FACTOR = 0.5f;
    final static float SCORING_DRIVE_POWER_FACTOR = 0.25f;  //TODO: CHANGE LATER

    //Gamepad Thresholds
    final static float ANALOG_STICK_THRESHOLD = .25f;
    final static float TRIGGER_THRESHOLD = .10f;

    //distance between wheels left to right of the Robot in inches.
    final static float ROBOT_TRACK_DISTANCE = 12.25f; //adjusted from observation.

    final static float FLOOR_RED_THRESHOLD = 0.5f;
    final static float FLOOR_BLUE_THRESHOLD = 0.5f;

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

    enum FilterJewelColorEnum {
        BLUE,
        RED,
        UNKNOWN
    }


    // Direction of movement for autonomous
    enum DirectionEnum {
        Forward, Backward, SidewaysLeft, SidewaysRight
    }

    // Direction of turning
    enum TurnDirectionEnum {
        Clockwise, Counterclockwise
    }

    enum JewelOrderEnum {
        BLUE_RED,
        RED_BLUE,
        UNKNOWN
    }
}
