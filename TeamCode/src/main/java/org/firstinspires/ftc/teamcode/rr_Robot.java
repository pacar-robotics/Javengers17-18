package org.firstinspires.ftc.teamcode;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_HOLDER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.rr_Constants.DEBUG_LEVEL;
import static org.firstinspires.ftc.teamcode.rr_Constants.ENCODER_COUNT_PER_DEGREE_TURN;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.GENERIC_TIMER;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_ARM_DOWN_PUSH;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_ARM_UP;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_COLOR_DIFFERENTIAL_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_COLOR_FILTER_COUNT;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_COLOR_LUMINOSITY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_PUSHER_LEFT;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_PUSHER_NEUTRAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_PUSHER_RIGHT;
import static org.firstinspires.ftc.teamcode.rr_Constants.LEFT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.rr_Constants.MAX_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_FRONT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_SIDE_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.rr_Constants.MIN_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_ENCODER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_PICKUP;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WRIST_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RIGHT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.ROBOT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_AUTONOMOUS_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;


public class rr_Robot {

//    JewelDetector jewelDetector;

    enum pictographType {
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN,
    }

    enum intakeStateEnum{
        RUNNING,
        STOPPED
    }

    enum cubePusherStateEnum{
        REST,
        PUSHED
    }

    rr_OpMode aOpMode;

    pictographType detectedPictograph = pictographType.UNKNOWN;

    intakeStateEnum intakeState = intakeStateEnum.STOPPED;

    cubePusherStateEnum cubePusherState=cubePusherStateEnum.REST;

    HardwareMap hwMap = null;

    //Motors
    private DcMotor motorArray[];

    //Servo
    private Servo jewelArm;
    private Servo jewelPusher;
    private Servo relicClaw;
    private Servo relicArm;
    private Servo relicWrist;

    private Servo trayFlipServo;

    private Servo cubePusherServo;
    private Servo cubeHolderServo; //servo that holds the cubes when tray is flipped and moves
                                   //out of the way when scoring.

    private ColorSensor leftJewelColorSensor;
    private ColorSensor rightJewelColorSensor;

    private DistanceSensor leftJewelRangeSensor;
    private DistanceSensor rightJewelRangeSensor;

    private DigitalChannel trayUpperLimit;
    private DigitalChannel trayLowerLimit;


    private ModernRoboticsI2cRangeSensor intakeRightRangeSensor;

    private  BNO055IMU.Parameters parameters;
    private BNO055IMU imu; //bosch imu embedded in the Rev Expansion Hub.
    Orientation angles; //part of IMU processing state
    Acceleration gravity; //part of IMU processing state

    //Variables for Ramped Power
    private double prevFLVelocity = 0.0f;
    private double prevFRVelocity = 0.0f;
    private double prevBLVelocity = 0.0f;
    private double prevBRVelocity = 0.0f;

    int trayHeightPosition = rr_Constants.TRAY_HEIGHT_COLLECTION_POSITION;
    float trayFlipPosition = rr_Constants.TRAY_FLIP_COLLECTION_POSITION;
    float relicArmPosition=RELIC_ARM_INIT;
    float relicWristPosition=RELIC_WRIST_INIT;
    float relicClawPosition=RELIC_CLAW_INIT;

    float cubePusherPosition= CUBE_PUSHER_RESTED_POSITION;
    float cubeHolderPosition=CUBE_HOLDER_INIT_POSITION;

    private ElapsedTime period = new ElapsedTime();

    public static final String TAG = "Vuforia VuMark Sample";
    VuforiaLocalizer vuforia;

    public rr_Robot(rr_OpMode aOpMode) throws InterruptedException {
        this.aOpMode = aOpMode;
    }


    /***********************************************
     *
     *     INITIALIZE METHODS
     *
     ***********************************************/


    public void autonomousInit(rr_OpMode aOpMode, HardwareMap ahwMap) throws InterruptedException {

        aOpMode.telemetry.setAutoClear(false); //useful to see the debug values stay on screen

        aOpMode.DBG("in Robot init");
        hwMap = ahwMap;

        //Instantiate motorArray

        motorArray = new DcMotor[10];

        //Initialize Drive Motors
        initDriveMotors(aOpMode);

        //initialize intake range sensor.
        initIntakeSensors(aOpMode);

       //setup the Bosch IMU
        setupBoschIMU(aOpMode); //only needed once per program run.

        //Initialize Relic Arm
        initRelic(aOpMode);
        //initRelicArmSensors(aOpMode);

        //Initialize Jewel Arm
        initJewelServos(aOpMode);

        initTrayServo(aOpMode);

        initCubePusherServo(aOpMode);

        initCubeHolderServo(aOpMode);


        //initialize trayMotor
        initTrayMotor(aOpMode);
        initTraySensors(aOpMode);
        
        aOpMode.DBG("Exiting Robot init");
    }

    public void teleopInit(rr_OpMode aOpMode, HardwareMap ahwMap) throws InterruptedException {
        aOpMode.telemetry.setAutoClear(false); //useful to see the debug values stay on screen

        aOpMode.DBG("in Robot init");
        hwMap = ahwMap;

        //Instantiate motorArray
        motorArray = new DcMotor[10];

        //setup and initialize the gyro.

        setupBoschIMU(aOpMode);

        //Initialize Drive Motors
        initDriveMotors(aOpMode);

        //initialize Intake Motors
        initIntakeMotors(aOpMode);

        //initialize intake range sensor.
        initIntakeSensors(aOpMode);


        //Initialize Relic Arm
       initRelic(aOpMode);
        //initRelicArmSensors(aOpMode);

        //Initialize Jewel Arm
        //initJewelSensors(aOpMode);
        initJewelServos(aOpMode);
        //setJewelPusherPosition(JEWEL_PUSHER_NEUTRAL);

        initTrayServo(aOpMode);

        initCubePusherServo(aOpMode);

        initCubeHolderServo(aOpMode);

        //initialize trayMotor

        initTrayMotor(aOpMode);

        initTraySensors(aOpMode);

        //initialize the tray to the collection position
        initTrayPosition(aOpMode);





        aOpMode.DBG("Exiting Robot init");
    }


    public void initTrayServo(rr_OpMode aOpMode) throws InterruptedException{

        trayFlipServo = hwMap.get(Servo.class, "servo_tray_flip");
        Thread.sleep(100);
        trayFlipServo.setPosition(TRAY_FLIP_COLLECTION_POSITION);
        Thread.sleep(250);
    }

    public void initCubePusherServo(rr_OpMode aOpMode) throws InterruptedException{
        cubePusherServo = hwMap.get(Servo.class, "servo_cube_pusher");
        Thread.sleep(100);
        setCubePusherPosition(aOpMode, CUBE_PUSHER_INIT_POSITION);
    }

    public void initCubeHolderServo(rr_OpMode aOpMode) throws InterruptedException{
        cubeHolderServo = hwMap.get(Servo.class, "servo_cube_holder");
        Thread.sleep(250);
        setCubeHolderPosition(aOpMode, CUBE_HOLDER_INIT_POSITION);
    }


    public void initDriveMotors(rr_OpMode aOpMode) throws InterruptedException {
        //Map Motors


        motorArray[FRONT_LEFT_MOTOR] = hwMap.get(DcMotor.class, "motor_front_left");
        motorArray[FRONT_RIGHT_MOTOR] = hwMap.get(DcMotor.class, "motor_front_right");
        motorArray[BACK_LEFT_MOTOR] = hwMap.get(DcMotor.class, "motor_back_left");
        motorArray[BACK_RIGHT_MOTOR] = hwMap.get(DcMotor.class, "motor_back_right");

        //Set the Direction of Motors
        motorArray[FRONT_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[FRONT_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[BACK_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[BACK_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);

        motorArray[INTAKE_LEFT_MOTOR]=hwMap.get(DcMotor.class,"motor_left_intake");
        motorArray[INTAKE_RIGHT_MOTOR]=hwMap.get(DcMotor.class,"motor_right_intake");
        motorArray[INTAKE_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[INTAKE_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);



        // Set all base motors to zero power
        stopBaseMotors(aOpMode);
    }


    public void initIntakeMotors(rr_OpMode aOpMode) throws InterruptedException {
        //Map Intake Motors
        motorArray[INTAKE_LEFT_MOTOR]=hwMap.get(DcMotor.class,"motor_left_intake");
        motorArray[INTAKE_RIGHT_MOTOR]=hwMap.get(DcMotor.class,"motor_right_intake");

        motorArray[INTAKE_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
        motorArray[INTAKE_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initTrayMotor(rr_OpMode aOpMode) throws InterruptedException {
        //Map Tray Motor
        motorArray[TRAY_LIFT_MOTOR]=hwMap.get(DcMotor.class,"motor_tray_lift");
        motorArray[TRAY_LIFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void initIntakeSensors(rr_OpMode aOpMode) throws InterruptedException{
        intakeRightRangeSensor=hwMap.get(ModernRoboticsI2cRangeSensor.class, "RightIntakeRangeSensor");
    }

    public void initRelic(rr_OpMode aOpMode) throws InterruptedException {
        motorArray[RELIC_WINCH_MOTOR] = hwMap.get(DcMotor.class, "motor_relic_slide");
        relicClaw = hwMap.get(Servo.class, "servo_relic_claw");
        relicArm = hwMap.get(Servo.class, "servo_relic_arm");
        relicWrist = hwMap.get(Servo.class, "servo_relic_wrist");

        setRelicClawPosition(RELIC_CLAW_INIT);
        setRelicArmPosition(RELIC_ARM_INIT);
        setRelicWristPosition(RELIC_WRIST_INIT);
    }

    public void initJewelServos(rr_OpMode aOpMode) throws InterruptedException {
        jewelArm = hwMap.get(Servo.class, "servo_jewel_arm");
        setJewelArmUp();
    }


    public void initTraySensors(rr_OpMode aOpMode) throws InterruptedException {
        trayUpperLimit = hwMap.get(DigitalChannel.class, "tray_upper_limit");
        trayLowerLimit = hwMap.get(DigitalChannel.class, "tray_lower_limit");

        trayUpperLimit.setMode(DigitalChannel.Mode.INPUT);
        trayLowerLimit.setMode(DigitalChannel.Mode.INPUT);
    }


    /***********************************************
     *
     *     MOTOR METHODS
     *
     ***********************************************/


    /**
     * Sets the power of the motor
     *
     * @param aOpMode   an object of the rr_OpMode class
     * @param motorName name of the motor
     * @param power     desired motor power
     * @throws InterruptedException
     */
    public void setPower(rr_OpMode aOpMode, int motorName, float power)
            throws InterruptedException {

        motorArray[motorName].setPower(power);
    }

    /**
     * Sets the runMode of the motor (with or without encoders)
     *
     * @param aOpMode   an object of the rr_OpMode class
     * @param motorName name of the motor
     * @param runMode   motor mode
     * @throws InterruptedException
     */
    public void setMotorMode(rr_OpMode aOpMode, int motorName,
                             DcMotor.RunMode runMode)
            throws InterruptedException {

        motorArray[motorName].setMode(runMode);
    }

    public int getMotorPosition(rr_OpMode aOpMode, int MotorNumber) {
        return motorArray[MotorNumber].getCurrentPosition();
    }

    public double getMotorPower(rr_OpMode aOpMode, int MotorNumber) {
        return motorArray[MotorNumber].getPower();
    }


    /***********************************************
     *
     *     SENSOR METHODS
     *
     ***********************************************/


    public float getBoschGyroSensorHeading(rr_OpMode aOpMode) throws InterruptedException {
        //grab the current heading from the IMU.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //reverse sense of the heading to match legacy code
        aOpMode.DBG("Heading:" + -angles.firstAngle);
        return -angles.firstAngle;



    }


    protected void setBoschGyroZeroYaw(rr_OpMode aOpMode) throws InterruptedException {
        initializeBoschIMU(aOpMode);
    }

    protected void setupBoschIMU(rr_OpMode aOpMode) throws InterruptedException {
        aOpMode.DBG("Starting Setup Bosch Gyro");
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        //parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu"); //get a new reference
        // to the IMU class. This should cause garbage collection of the old object.
        // Also should set the system up for the new calibrated values.
        imu.initialize(parameters);
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Thread.sleep(500);
        aOpMode.DBG("End Setup Bosch Gyro");
    }

    protected void initializeBoschIMU(rr_OpMode aOpMode) throws InterruptedException {
        aOpMode.Echo("Starting Initialize Bosch Gyro");

        imu.initialize(parameters);
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Thread.sleep(500);
        aOpMode.Echo("End Initialize Bosch Gyro");
    }


    /***********************************************
     *
     *     CONTROL OF WHEELS
     *
     ***********************************************/


    /**
     * Runs robot to a specific position. Can be called by other, more specific methods to move forwards, backwards or sideways.
     *
     * @param aOpMode     an object of the rr_OpMode class
     * @param fl_Power    front right motor power
     * @param fr_Power    front left motor power
     * @param bl_Power    back left motor power
     * @param br_Power    back right motor power
     * @param fl_Position front left motor cubeClawPos
     * @param fr_Position front left motor cubeClawPos
     * @param bl_Position back left motor cubeClawPos
     * @param br_Position back right motor cubeClawPos
     */
    public void runRobotToPosition(rr_OpMode aOpMode, float fl_Power, float fr_Power,
                                   float bl_Power, float br_Power, int fl_Position,
                                   int fr_Position, int bl_Position, int br_Position,
                                   boolean isRampedPower)
            throws InterruptedException {

        double rampedPower;

        //reset motor encoders
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(50);

        //sets all motors to run to a position
        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set targets
        motorArray[FRONT_LEFT_MOTOR].setTargetPosition(fl_Position);
        motorArray[FRONT_RIGHT_MOTOR].setTargetPosition(fr_Position);
        motorArray[BACK_LEFT_MOTOR].setTargetPosition(bl_Position);
        motorArray[BACK_RIGHT_MOTOR].setTargetPosition(br_Position);
        Thread.sleep(50);


        if (isRampedPower) {
            //sets the the power of all motors
            //since we are ramping up, start at the lowest power allowed.
            setPower(aOpMode, FRONT_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(aOpMode, FRONT_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(aOpMode, BACK_LEFT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);
            setPower(aOpMode, BACK_RIGHT_MOTOR, MOTOR_LOWER_POWER_THRESHOLD);

        } else {
            setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power * LEFT_MOTOR_TRIM_FACTOR);
            setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power * RIGHT_MOTOR_TRIM_FACTOR);
            setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power * LEFT_MOTOR_TRIM_FACTOR);
            setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power * RIGHT_MOTOR_TRIM_FACTOR);
        }


        aOpMode.reset_timer_array(GENERIC_TIMER);

        while (baseMotorsAreBusy() && (aOpMode.time_elapsed_array(GENERIC_TIMER) < MAX_MOTOR_LOOP_TIME) &&
                (Math.abs(fl_Position - motorArray[FRONT_LEFT_MOTOR].getCurrentPosition())
                        >= MECANUM_WHEEL_ENCODER_MARGIN)) {
            //wait until motors havce completed movement or timed out.
            //report motor positions for debugging

            //adjust the motor speeds by adjusting Power proportional to distance that needs to be travelled.


            //Ramped Move block formula:
            //RP=PMax(1-4*(0.5-DT/DD)^2)
            //where RP=Ramped Power, PMax is maximum power available, DT=Distance Travelled, DD=Distance to be travelled
            //fl_position (target for the front left motor in encoder clicks can be taken as the proxy for all motors.

            //using fl_power as proxy for all wheel power, the sign is not relevant in this runmode.

            float rampedPowerRaw = (float) (fl_Power * (1 - 4 * (Math.pow((0.5f -
                    Math.abs((motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() * 1.0f) / fl_Position)), 2.0f))));

            //use another variable to check and adjust power limits, so we can display raw power values.
            if (isRampedPower) {
                rampedPower = rampedPowerRaw;
            } else {
                rampedPower = fl_Power; //as proxy for all power.
            }

            if (Math.signum(fl_Position) != Math.signum(fr_Position)) {
                //we are moving sideways, since the front left and right wheels are rotating in opposite directions
                //we should check against sideways limits.
                //check for upper and lower limits.
                if (rampedPower > MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT) {
                    rampedPower = MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
                }
                if (rampedPower < MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT) {
                    rampedPower = MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
                }
            } else {
                //else we are moving forward
                //check for upper and lower limits.
                if (rampedPower > MOTOR_RAMP_FB_POWER_UPPER_LIMIT) {
                    rampedPower = MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
                }
                if (rampedPower < MOTOR_RAMP_FB_POWER_LOWER_LIMIT) {
                    rampedPower = MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
                }
            }

            //apply the new power values.
            //sets the the power of all motors

            //in this runmode, the power does not control direction but the sign of the target position does.

            motorArray[FRONT_LEFT_MOTOR].setPower(rampedPower * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[FRONT_RIGHT_MOTOR].setPower(rampedPower * RIGHT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_LEFT_MOTOR].setPower(rampedPower * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_RIGHT_MOTOR].setPower(rampedPower * RIGHT_MOTOR_TRIM_FACTOR);

            if (DEBUG_LEVEL > 1) {
                aOpMode.telemetryAddData("Motor FL", "Values", ":" + motorArray[FRONT_LEFT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Motor FR", "Values", ":" + motorArray[FRONT_RIGHT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Motor BL", "Values", ":" + motorArray[BACK_LEFT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Motor BR", "Values", ":" + motorArray[BACK_RIGHT_MOTOR].getCurrentPosition());
                aOpMode.telemetryAddData("Raw Ramped Power", "Values", ":" + rampedPowerRaw);
                aOpMode.telemetryAddData("Ramped Power ", "Values", ":" + rampedPower);
                aOpMode.telemetryAddData("fl_position", "Values", ":" + fl_Position);
                aOpMode.telemetryAddData("DTraveled/DTarget", "Values", ":" +
                        Math.abs(motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() / fl_Position));
                aOpMode.telemetryAddData("Squared Values", "Values", ":" +
                        Math.pow((0.5f -
                                Math.abs((motorArray[FRONT_LEFT_MOTOR].getCurrentPosition() * 1.0f) / fl_Position)), 2.0f));


                aOpMode.telemetryUpdate();
            }
            aOpMode.idle();
        }
        stopBaseMotors(aOpMode);

    }


    /**
     * Runs robot to a specific cubeClawPos while driving forwards or backwards
     *
     * @param aOpMode       an object of the rr_OpMode class
     * @param distance      distance each wheel will go in inches
     * @param power         desired power of motor
     * @param isRampedPower ramps power to prevent jerking if true
     * @throws InterruptedException
     */
    public void moveRobotToPositionFB(rr_OpMode aOpMode, float distance,
                                      float power, boolean isRampedPower)
            throws InterruptedException {
        //we need to store the encoder target cubeClawPos
        int targetPosition;
        //calculate target cubeClawPos from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //using the generic method with all powers set to the same value and all positions set to the same cubeClawPos
        runRobotToPosition(aOpMode, power, power, power, power,
                targetPosition, targetPosition, targetPosition, targetPosition, isRampedPower);
    }

    /**
     * Runs robot to a specific cubeClawPos while driving sideways
     *
     * @param aOpMode  an object of the rr_OpMode class
     * @param distance distance wheels will go in inches
     * @param power    generic power of the motors (positive = left, negative = right)
     */
    public void moveRobotToPositionSideways(rr_OpMode aOpMode, float distance,
                                            float power, boolean isRampedPower)
            throws InterruptedException {
        //we need to
        //store the encoder target cubeClawPos
        int targetPosition;
        //calculate target cubeClawPos from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //using the generic method with all powers set to the same value and all positions set to the same cubeClawPos
        runRobotToPosition(aOpMode, power, power, power, power,
                -targetPosition, targetPosition, targetPosition, -targetPosition, isRampedPower);
    }

    /**
     * Runs motors without a specified duration.
     * Can be called by a more specific method to move forwards, backwards or sideways.
     *
     * @param aOpMode  object of rr_OpMode class
     * @param fl_Power power of front left motor
     * @param fr_Power power of front right motor
     * @param bl_Power power of back left motor
     * @param br_Power power of back right motor
     */
    public void runMotors(rr_OpMode aOpMode, float fl_Power, float fr_Power, float bl_Power, float br_Power)
            throws InterruptedException {

        motorArray[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(50);


        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power);
        setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power);
        setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power);
    }

    public void runRampedMotors(rr_OpMode aOpMode, float fl_Power, float fr_Power, float bl_Power, float br_Power)
            throws InterruptedException {

        motorArray[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(50);

        fl_Power = (float) (fl_Power + prevFLVelocity) / 2f;
        fr_Power = (float) (fr_Power + prevFRVelocity) / 2f;
        bl_Power = (float) (bl_Power + prevBLVelocity) / 2f;
        br_Power = (float) (br_Power + prevBRVelocity) / 2f;

        prevFLVelocity = fl_Power;
        prevFRVelocity = fr_Power;
        prevBLVelocity = bl_Power;
        prevBRVelocity = br_Power;

        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, fl_Power);
        setPower(aOpMode, FRONT_RIGHT_MOTOR, fr_Power);
        setPower(aOpMode, BACK_LEFT_MOTOR, bl_Power);
        setPower(aOpMode, BACK_RIGHT_MOTOR, br_Power);

    }


    /**
     * Runs motors forwards or backwards.
     *
     * @param power each motor will run at the same float value
     */
    public void runMotorsFB(rr_OpMode aOpMode, float power) throws InterruptedException {
        runMotors(aOpMode, power, power, power, power);
    }

    public void runMotorsUsingEncoders(rr_OpMode aOpMode, float fl_Power, float fr_Power, float bl_Power, float br_Power)
            throws InterruptedException {

        motorArray[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(50);


        //sets the the power of all motors
        setPower(aOpMode, FRONT_LEFT_MOTOR, limit_power(aOpMode, fl_Power));
        setPower(aOpMode, FRONT_RIGHT_MOTOR, limit_power(aOpMode, fr_Power));
        setPower(aOpMode, BACK_LEFT_MOTOR, limit_power(aOpMode, bl_Power));
        setPower(aOpMode, BACK_RIGHT_MOTOR, limit_power(aOpMode, br_Power));

        Thread.sleep(50);
    }

    /**
     * Stops all wheel motors
     *
     * @param aOpMode an object of the rr_OpMode class
     * @throws InterruptedException
     */
    public void stopBaseMotors(rr_OpMode aOpMode) throws InterruptedException {

        motorArray[FRONT_LEFT_MOTOR].setPower(0);
        motorArray[FRONT_RIGHT_MOTOR].setPower(0);
        motorArray[BACK_LEFT_MOTOR].setPower(0);
        motorArray[BACK_RIGHT_MOTOR].setPower(0);
    }


    public boolean baseMotorsAreBusy() {
        return (motorArray[FRONT_LEFT_MOTOR].isBusy() || motorArray[FRONT_RIGHT_MOTOR].isBusy() ||
                motorArray[BACK_LEFT_MOTOR].isBusy() || motorArray[BACK_RIGHT_MOTOR].isBusy());
    }

    public void universalMoveRobot(rr_OpMode aOpMode, double xAxisVelocity,
                                   double yAxisVelocity)
            throws InterruptedException {
        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double trackDistanceAverage = (MECANUM_WHEEL_FRONT_TRACK_DISTANCE +
                MECANUM_WHEEL_SIDE_TRACK_DISTANCE) / 2.0f;


        //calculate velocities at each wheel.

        /**
        //blend with prev velocities to smooth out start

        fl_velocity = ((yAxisVelocity + xAxisVelocity) + prevFLVelocity) / 2;

        fr_velocity = ((yAxisVelocity - xAxisVelocity) + prevFRVelocity) / 2;

        bl_velocity = ((yAxisVelocity - xAxisVelocity) + prevBLVelocity) / 2;

        br_velocity = ((yAxisVelocity + xAxisVelocity) + prevBRVelocity) / 2;
        *
        **/

        //ignores rotational velocity
        fl_velocity = yAxisVelocity + xAxisVelocity ;

        fr_velocity = yAxisVelocity - xAxisVelocity ;

        bl_velocity = yAxisVelocity - xAxisVelocity;

        br_velocity = yAxisVelocity + xAxisVelocity ;

        //save these in variables that are part of vvRobot to be used in next cycle.

        prevFLVelocity = fl_velocity;
        prevFRVelocity = fr_velocity;
        prevBLVelocity = bl_velocity;
        prevBRVelocity = br_velocity;


        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //apply specific powers to motors to get desired movement


        motorArray[FRONT_LEFT_MOTOR].setPower(fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[FRONT_RIGHT_MOTOR].setPower(fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_LEFT_MOTOR].setPower(bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_RIGHT_MOTOR].setPower(br_velocity * RIGHT_MOTOR_TRIM_FACTOR);
    }

    public void universalMoveRobotWithCondition(rr_OpMode aOpMode, double xAxisVelocity,
                                                double yAxisVelocity, double rotationalVelocity,
                                                long duration, rr_OpMode.StopCondition condition,
                                                boolean isPulsed, long pulseWidthDuration, long pulseRestDuration)
            throws InterruptedException {
        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double trackDistanceAverage = (MECANUM_WHEEL_FRONT_TRACK_DISTANCE +
                MECANUM_WHEEL_SIDE_TRACK_DISTANCE) / 2.0f;


        //calculate velocities at each wheel.

        fl_velocity = yAxisVelocity + xAxisVelocity - rotationalVelocity *
                trackDistanceAverage;

        fr_velocity = yAxisVelocity - xAxisVelocity + rotationalVelocity *
                trackDistanceAverage;

        bl_velocity = yAxisVelocity - xAxisVelocity - rotationalVelocity *
                trackDistanceAverage;

        br_velocity = yAxisVelocity + xAxisVelocity + rotationalVelocity *
                trackDistanceAverage;

        //reset all encoders.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Thread.sleep(50);

        //switch to RUN_WITH_ENCODERS to normalize for speed.

        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //start slow to prevent skid.
        //may replace with ramp to improve performance.

        motorArray[FRONT_LEFT_MOTOR].setPower(Math.abs(fl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[FRONT_RIGHT_MOTOR].setPower(Math.abs(fr_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fr_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_LEFT_MOTOR].setPower(Math.abs(bl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(bl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_RIGHT_MOTOR].setPower(Math.abs(br_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(br_velocity) * MOTOR_LOWER_POWER_THRESHOLD : br_velocity * RIGHT_MOTOR_TRIM_FACTOR);

        //

        aOpMode.reset_timer_array(GENERIC_TIMER);
        //stop 100 ms before end
        while ((aOpMode.time_elapsed_array(GENERIC_TIMER) < (duration - 100)) &&
                (!condition.stopCondition(aOpMode))) {

            //condition will return true when it reaches state meant to stop movement

            //apply specific powers to motors to get desired movement
            //wait till duration is complete.
            motorArray[FRONT_LEFT_MOTOR].setPower(fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[FRONT_RIGHT_MOTOR].setPower(fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_LEFT_MOTOR].setPower(bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
            motorArray[BACK_RIGHT_MOTOR].setPower(br_velocity * RIGHT_MOTOR_TRIM_FACTOR);

            if (isPulsed) {
                //run the motors for the pulseWidthDuration
                //by sleeping, we let the motors that are running to continue to run
                Thread.sleep(pulseWidthDuration);

                //stop motors
                stopBaseMotors(aOpMode);
                //pause for pulseRestDuration
                Thread.sleep(pulseRestDuration);
                //this allows the robot to move slowly and gives the sensor time to read
            }
            aOpMode.idle();
        }

        //end slow
        motorArray[FRONT_LEFT_MOTOR].setPower(Math.abs(fl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[FRONT_RIGHT_MOTOR].setPower(Math.abs(fr_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(fr_velocity) * MOTOR_LOWER_POWER_THRESHOLD : fr_velocity * RIGHT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_LEFT_MOTOR].setPower(Math.abs(bl_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(bl_velocity) * MOTOR_LOWER_POWER_THRESHOLD : bl_velocity * LEFT_MOTOR_TRIM_FACTOR);
        motorArray[BACK_RIGHT_MOTOR].setPower(Math.abs(br_velocity) > MOTOR_LOWER_POWER_THRESHOLD ?
                Math.signum(br_velocity) * MOTOR_LOWER_POWER_THRESHOLD : br_velocity * RIGHT_MOTOR_TRIM_FACTOR);

        aOpMode.reset_timer_array(GENERIC_TIMER);
        //stop 100 ms before end
        while (aOpMode.time_elapsed_array(GENERIC_TIMER) < 100) {
            aOpMode.idle();
        }


        //stop all motors
        stopBaseMotors(aOpMode);
    }

    public void turnAbsoluteBoschGyroDegreesAuto(rr_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException {
        //clockwise is represented by clockwise numbers.
        //counterclockwise by negative angle numbers in degrees.
        //the fieldReferenceDegrees parameters measures degrees off the initial reference frame when the robot is started and the gyro is
        //calibrated.
        // >> IMPORTANT: This depends on the zIntegratedHeading not being altered by relative turns !!!

        //first take the absolute degrees and modulus down to 0 and 359.

        float targetDegrees = fieldReferenceDegrees % 360;

        //compare to the current gyro zIntegrated heading and store the result.
        //the Integrated zValue returned is positive for clockwise turns
        //read the heading and store it.
        float startingHeading = getBoschGyroSensorHeading(aOpMode);
        float turnDegrees = targetDegrees - startingHeading;

        //make the turn using encoders

        if (DEBUG) {
            aOpMode.telemetryAddData("targetDegrees", "Value",
                    ":" + targetDegrees);
            aOpMode.telemetryAddData("Starting Z", "Value",
                    ":" + startingHeading);
            aOpMode.telemetryAddData("Turn Degrees", "Value",
                    ":" + turnDegrees);

            aOpMode.telemetryUpdate();
        }

        //optimize the turn, so that direction of turn results in smallest turn needed.

        if (Math.abs(turnDegrees) > 180) {
            turnDegrees = Math.signum(turnDegrees) * -1 * (360 - Math.abs(turnDegrees));
        }

        turnUsingEncoders(aOpMode, Math.abs(turnDegrees), TURN_AUTONOMOUS_POWER_FACTOR,
                turnDegrees > 0 ? rr_Constants.TurnDirectionEnum.Clockwise :
                        rr_Constants.TurnDirectionEnum.Counterclockwise);

        float finalDegrees = getBoschGyroSensorHeading(aOpMode);
        Thread.sleep(50); //cooling off after gyro read to prevent error in next run.

        if (DEBUG) {
            aOpMode.telemetryAddData("New Bearing Degrees", "Value:",
                    ":" + finalDegrees);
            aOpMode.telemetryAddData("Turn Error Degrees", "Value:",
                    ":" + (targetDegrees - finalDegrees));
            aOpMode.telemetryUpdate();
        }

    }


    public void turnAbsoluteBoschGyroDegrees(rr_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException {
        //clockwise is represented by clockwise numbers.
        //counterclockwise by negative angle numbers in degrees.
        //the fieldReferenceDegrees parameters measures degrees off the initial reference frame when the robot is started and the gyro is
        //calibrated.
        // >> IMPORTANT: This depends on the zIntegratedHeading not being altered by relative turns !!!

        //first take the absolute degrees and modulus down to 0 and 359.

        float targetDegrees = fieldReferenceDegrees % 360;

        //compare to the current gyro zIntegrated heading and store the result.
        //the Integrated zValue returned is positive for clockwise turns
        //read the heading and store it.
        float startingHeading = getBoschGyroSensorHeading(aOpMode);
        float turnDegrees = targetDegrees - startingHeading;

        //make the turn using encoders

        if (DEBUG) {
            aOpMode.telemetryAddData("targetDegrees", "Value",
                    ":" + targetDegrees);
            aOpMode.telemetryAddData("Starting Z", "Value",
                    ":" + startingHeading);
            aOpMode.telemetryAddData("Turn Degrees", "Value",
                    ":" + turnDegrees);

            aOpMode.telemetryUpdate();
        }

        //optimize the turn, so that direction of turn results in smallest turn needed.

        if (Math.abs(turnDegrees) > 180) {
            turnDegrees = Math.signum(turnDegrees) * -1 * (360 - Math.abs(turnDegrees));
        }

        turnUsingEncoders(aOpMode, Math.abs(turnDegrees), TURN_POWER_FACTOR,
                turnDegrees > 0 ? rr_Constants.TurnDirectionEnum.Clockwise :
                        rr_Constants.TurnDirectionEnum.Counterclockwise);

        float finalDegrees = getBoschGyroSensorHeading(aOpMode);
        Thread.sleep(50); //cooling off after gyro read to prevent error in next run.

        if (DEBUG) {
            aOpMode.telemetryAddData("New Bearing Degrees", "Value:",
                    ":" + finalDegrees);
            aOpMode.telemetryAddData("Turn Error Degrees", "Value:",
                    ":" + (targetDegrees - finalDegrees));
            aOpMode.telemetryUpdate();
        }

    }
    public void  turnUsingEncodersWithoutRamped(rr_OpMode aOpMode, float angle, float power, rr_Constants.TurnDirectionEnum TurnDirection)
            throws InterruptedException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.

        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECANUM_WHEEL_DIAMETER * 360));

        switch (TurnDirection) {
            case Clockwise:
                runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance, false);
                break;
            case Counterclockwise:
                runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance, false);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }

    public void turnUsingEncoders(rr_OpMode aOpMode, float angle, float power, rr_Constants.TurnDirectionEnum TurnDirection)
            throws InterruptedException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.
/*
        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECANUM_WHEEL_DIAMETER * 360));

*/
        int turnDistance=Math.round(angle*ENCODER_COUNT_PER_DEGREE_TURN); //this should be a better calculation for turns.


        switch (TurnDirection) {
            case Clockwise:
                runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance, true);
                break;
            case Counterclockwise:
                runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance, true);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }


    public void setRelicWinchPower(float power) {
        motorArray[RELIC_WINCH_MOTOR].setPower(power);
    }

    public int getRelicWinchPosition() {
        return motorArray[RELIC_WINCH_MOTOR].getCurrentPosition();
    }

    public void setRelicArmGrab() throws InterruptedException {
        relicArm.setPosition(RELIC_ARM_PICKUP);
        Thread.sleep(100);
    }


    public void setRelicArmPosition(float position) {
        relicArmPosition=position;
        relicArm.setPosition(position);
    }

    public void setRelicWristPosition(float position) {
        relicWristPosition=position;
        relicWrist.setPosition(position);
    }


    public float getRelicArmPosition() {
        return (float) relicArm.getPosition();
    }

    public float getJewelArmPosition() {
        return (float) jewelArm.getPosition();
    }

    public void setRelicClawClosed() throws InterruptedException {
        relicClaw.setPosition(RELIC_CLAW_CLOSED);
        Thread.sleep(100);
    }

    public void setRelicClawOpen() throws InterruptedException {
        relicClaw.setPosition(RELIC_CLAW_OPEN+0.05);
        Thread.sleep(500);
        relicClaw.setPosition(RELIC_CLAW_OPEN);
        Thread.sleep(100);
    }

    public void setRelicClawPosition(float position) throws InterruptedException {
        relicClaw.setPosition(position);
    }

    public float getRelicClawPosition() throws InterruptedException {
        return (float) relicClaw.getPosition();
    }


    /***********************************************
     *
     *     CUBE CONTROL
     *
     ***********************************************/

    public void setTrayLiftPower(rr_OpMode aOpMode, float power) throws InterruptedException {
        setPower(aOpMode, TRAY_LIFT_MOTOR, power);
    }


    public boolean isTrayUpperLimitPressed() {
        return !trayUpperLimit.getState();
    }

    public boolean isTrayLowerLimitPressed() {

        return !trayLowerLimit.getState();
    }



    /***********************************************
     *
     *     JEWEL ARM CONTROL
     *
     ***********************************************/

    public void setJewelArmPosition(rr_OpMode aOpMode, float position) throws InterruptedException
    {
        jewelArm.setPosition(position);
        Thread.sleep(100);
    }

    public void setJewelArmPositionTest(float armPosition) throws InterruptedException {

        if (jewelArm.getPosition() > armPosition) {
            boolean isJewelArmAboveFinal = true;
            while (isJewelArmAboveFinal) {
                if (jewelArm.getPosition() > armPosition) {
                    jewelArm.setPosition(jewelArm.getPosition() - rr_Constants.JEWEL_ARM_INCREMENT);
                    Thread.sleep(rr_Constants.JEWEL_ARM_CYCLE);
                } else {
                    isJewelArmAboveFinal = false;
                }
            }
        } else {
            boolean isJewelArmBelowFinal = true;
            while (isJewelArmBelowFinal) {
                if (jewelArm.getPosition() < armPosition) {
                    jewelArm.setPosition(jewelArm.getPosition() + rr_Constants.JEWEL_ARM_INCREMENT);
                    Thread.sleep(rr_Constants.JEWEL_ARM_CYCLE);
                } else {
                    isJewelArmBelowFinal = false;
                }
            }
        }
    }

    public void setJewelArmPosition (float armPosition) throws InterruptedException {
        jewelArm.setPosition(armPosition);
        Thread.sleep(50);
    }

    public void setJewelPusherPosition(float armPosition) throws InterruptedException {
        jewelPusher.setPosition(armPosition);
        Thread.sleep(50);
    }

    public void pushRightJewel() throws InterruptedException {
        jewelPusher.setPosition(JEWEL_PUSHER_RIGHT);
        Thread.sleep(500);
    }

    public void pushLeftJewel() throws InterruptedException {
        jewelPusher.setPosition(JEWEL_PUSHER_LEFT);
        Thread.sleep(500);
    }

    public void setJewelPusherNeutral() throws InterruptedException {
        jewelPusher.setPosition(JEWEL_PUSHER_NEUTRAL);
        Thread.sleep(500);
    }

    public void setJewelArmUp() throws InterruptedException {
        jewelArm.setPosition(JEWEL_ARM_UP);
        Thread.sleep(100);
    }


    public void setJewelArmDownPush() throws InterruptedException {
        setJewelArmPosition(JEWEL_ARM_DOWN_PUSH);
    }

    public double getLeftJewelRange(rr_OpMode aOpMode) {
        return leftJewelRangeSensor.getDistance(DistanceUnit.CM);
    }

    public double getRightJewelRange(rr_OpMode aOpMode) {
        return rightJewelRangeSensor.getDistance(DistanceUnit.CM);
    }

    // Applies filter to reduce noise for left range sensor
    public double getFilteredLeftJewelRangeReading(rr_OpMode aOpMode) throws InterruptedException {

        double leftJewelRangeReadingsArray[] = new double[10];

        for (int i = 0; i < 10; i++) {
            leftJewelRangeReadingsArray[i] = getLeftJewelRange(aOpMode);
            Thread.sleep(30);
            if (leftJewelRangeReadingsArray[i] == 0) {
                i--;
            }
        }

        Arrays.sort(leftJewelRangeReadingsArray);

        aOpMode.telemetryAddData("Left Jewel Distance", "Readings Array", Arrays.toString(leftJewelRangeReadingsArray));
        aOpMode.telemetryAddData("Left Jewel Distance", "Median Value", "Median" + leftJewelRangeReadingsArray[4]);

        aOpMode.telemetryUpdate();

        return leftJewelRangeReadingsArray[4];
    }

    // Applies filter to reduce noise for left range sensor
    public double getFilteredRightJewelRangeReading(rr_OpMode aOpMode) throws InterruptedException {

        double rightJewelRangeReadingsArray[] = new double[10];

        for (int i = 0; i < 10; i++) {
            rightJewelRangeReadingsArray[i] = getRightJewelRange(aOpMode);
            Thread.sleep(30);
            if (rightJewelRangeReadingsArray[i] == 0) {
                i--;
            }
        }

        Arrays.sort(rightJewelRangeReadingsArray);

        aOpMode.telemetryAddData("Right Jewel Distance", "Readings Array", Arrays.toString(rightJewelRangeReadingsArray));
        aOpMode.telemetryAddData("Right Jewel Distance", "Median Value", "Median" + rightJewelRangeReadingsArray[4]);

        aOpMode.telemetryUpdate();

        return rightJewelRangeReadingsArray[4];
    }

    // Applies filter to reduce noise for left jewel color sensor readings
    public rr_Constants.FilterJewelColorEnum getFilteredLeftJewelColor(rr_OpMode aOpMode) throws InterruptedException {

        float colorSensorReading = 0;
        float leftJewelRedArray[] = new float[(JEWEL_COLOR_FILTER_COUNT)];
        float leftJewelBlueArray[] = new float[(JEWEL_COLOR_FILTER_COUNT)];

        for (int i = 0; i < (JEWEL_COLOR_FILTER_COUNT - 1); i++) {
            leftJewelRedArray[i] = leftJewelColorSensor.red();
            Thread.sleep(30);
            if (leftJewelRedArray[i] == 0) {
                i--;
            }
        }

        for (int i = 0; i < (JEWEL_COLOR_FILTER_COUNT - 1); i++) {
            leftJewelBlueArray[i] = leftJewelColorSensor.blue();
            Thread.sleep(30);
            if (leftJewelBlueArray[i] == 0) {
                i--;
            }
        }


        Arrays.sort(leftJewelBlueArray);
        Arrays.sort(leftJewelRedArray);


        aOpMode.telemetryAddData("Right Jewel", "Blue Readings Array", Arrays.toString(leftJewelBlueArray));
        aOpMode.telemetryAddData("Right Jewel", "Red Readings Array", Arrays.toString(leftJewelRedArray));
        aOpMode.telemetryAddData("Right Jewel", "Blue Median Value", "Median" + leftJewelBlueArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2]);
        aOpMode.telemetryAddData("Right Jewel", "Red Median Value", "Median" + leftJewelRedArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2]);

        aOpMode.telemetryUpdate();

        if (leftJewelBlueArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] > (leftJewelRedArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] + JEWEL_COLOR_DIFFERENTIAL_THRESHOLD)) {
            return rr_Constants.FilterJewelColorEnum.BLUE;
        } else if (leftJewelRedArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] > (leftJewelBlueArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] + JEWEL_COLOR_DIFFERENTIAL_THRESHOLD)) {
            return rr_Constants.FilterJewelColorEnum.RED;
        } else {
            return rr_Constants.FilterJewelColorEnum.UNKNOWN;
        }
    }

    // Applies filter to reduce noise for right jewel color sensor readings
    public rr_Constants.FilterJewelColorEnum getFilteredRightJewelColor(rr_OpMode aOpMode) throws InterruptedException {

        float colorSensorReading = 0;
        float rightJewelRedArray[] = new float[JEWEL_COLOR_FILTER_COUNT];
        float rightJewelBlueArray[] = new float[JEWEL_COLOR_FILTER_COUNT];

        for (int i = 0; i < JEWEL_COLOR_FILTER_COUNT; i++) {
            rightJewelRedArray[i] = rightJewelColorSensor.red();

            Thread.sleep(30);
            if (rightJewelRedArray[i] == 0) {
                i--;
            }
        }

        for (int i = 0; i < JEWEL_COLOR_FILTER_COUNT; i++) {
            rightJewelBlueArray[i] = rightJewelColorSensor.blue();

            Thread.sleep(30);
            if (rightJewelBlueArray[i] == 0) {
                i--;
            }
        }

        Arrays.sort(rightJewelBlueArray);
        Arrays.sort(rightJewelRedArray);


        aOpMode.telemetryAddData("Right Jewel", "Blue Readings Array", Arrays.toString(rightJewelBlueArray));
        aOpMode.telemetryAddData("Right Jewel", "Red Readings Array", Arrays.toString(rightJewelRedArray));
        aOpMode.telemetryAddData("Right Jewel", "Blue Median Value", "Median" + rightJewelBlueArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2]);
        aOpMode.telemetryAddData("Right Jewel", "Red Median Value", "Median" + rightJewelRedArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2]);

        aOpMode.telemetryUpdate();

        if (rightJewelBlueArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] > (rightJewelRedArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] + JEWEL_COLOR_DIFFERENTIAL_THRESHOLD)) {
            return rr_Constants.FilterJewelColorEnum.BLUE;
        } else if (rightJewelRedArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] > (rightJewelBlueArray[(JEWEL_COLOR_FILTER_COUNT - 1)/2] + JEWEL_COLOR_DIFFERENTIAL_THRESHOLD)) {
            return rr_Constants.FilterJewelColorEnum.RED;
        } else {
            return rr_Constants.FilterJewelColorEnum.UNKNOWN;
        }
    }


    public rr_Constants.JewelColorEnum getJewelLeftColor(rr_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(25);
        if(getJewelLeftLuminosity(aOpMode)>JEWEL_COLOR_LUMINOSITY_THRESHOLD) {
            if (getFilteredLeftJewelColor(aOpMode) == rr_Constants.FilterJewelColorEnum.RED) {

                aOpMode.telemetryAddData("Color", "Red", "Left Red Detected");
                aOpMode.telemetryUpdate();
                return rr_Constants.JewelColorEnum.RED;
            }
            if (getFilteredLeftJewelColor(aOpMode) == rr_Constants.FilterJewelColorEnum.BLUE) {
                aOpMode.telemetryAddData("Color", "Blue", "Left Blue Detected");
                aOpMode.telemetryUpdate();
                return rr_Constants.JewelColorEnum.BLUE;
            }
        }
        aOpMode.telemetryAddData("Color", "Unknown", "No Color Detected");
        aOpMode.telemetryUpdate();
        return rr_Constants.JewelColorEnum.UNKNOWN;
    }

    public rr_Constants.JewelColorEnum getJewelRightColor(rr_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(25);
        if(getJewelRightLuminosity(aOpMode)>JEWEL_COLOR_LUMINOSITY_THRESHOLD) {
            if (getFilteredRightJewelColor(aOpMode) == rr_Constants.FilterJewelColorEnum.RED) {
                aOpMode.telemetryAddData("Color", "Red", "Right Red Detected");
                aOpMode.telemetryUpdate();
                return rr_Constants.JewelColorEnum.RED;
            }
            if (getFilteredRightJewelColor(aOpMode) == rr_Constants.FilterJewelColorEnum.BLUE) {
                aOpMode.telemetryAddData("Color", "Blue", "Right Blue Detected");
                aOpMode.telemetryUpdate();

                return rr_Constants.JewelColorEnum.BLUE;
            }
        }

        aOpMode.telemetryAddData("Color", "Unknown", "No Color Detected");
        aOpMode.telemetryUpdate();
        return rr_Constants.JewelColorEnum.UNKNOWN;
    }

    public float getJewelLeftLuminosity(rr_OpMode aOpMode) {
        return leftJewelColorSensor.alpha();
    }

    public float getJewelRightLuminosity(rr_OpMode aOpMode) {
        return rightJewelColorSensor.alpha();
    }


    /**
     * Tests motors using debug statements
     *
     * @param aOpMode   an object of the rr_OpMode class
     * @param motorName name of the motor
     * @param power     desired motor power
     * @param duration  duration in seconds
     * @throws InterruptedException
     */
    public void testMotor(rr_OpMode aOpMode, int motorName, float power, long duration) throws InterruptedException {
        aOpMode.DBG("In test motor in vv_robot");

        aOpMode.DBG("after checkname assertion in vv_robot");

        //save the old run mode
        DcMotor.RunMode oldRunMode = motorArray[motorName].getMode();
        aOpMode.DBG("after getmode in vv_robot");

        //change mode to run without encoders

        motorArray[motorName].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        aOpMode.DBG("after runmode set in vv_robot");

        //set the power to motor
        motorArray[motorName].setPower(power);

        aOpMode.DBG("after power set in vv_robot");

        //reset the timer
        aOpMode.reset_timer_array(GENERIC_TIMER);
        while (aOpMode.time_elapsed_array(GENERIC_TIMER) < duration) {
            //wait till duration is complete.
            aOpMode.DBG("In motor loop in vv_robot");
            aOpMode.idle();
        }
        //stop the motor

        motorArray[motorName].setPower(0.0f);

        //restore old motor mode
        motorArray[motorName].setMode(oldRunMode);
    }


    public float limit_power(rr_OpMode aOpMode, float power) {
        //Check and limit the power being applied to motors during turns
        if (power == 0) {
            return 0;
        } else {
            return ((power < MIN_ROBOT_TURN_MOTOR_VELOCITY ? MIN_ROBOT_TURN_MOTOR_VELOCITY :
                    (power > MAX_ROBOT_TURN_MOTOR_VELOCITY ? MAX_ROBOT_TURN_MOTOR_VELOCITY : power)));
        }
    }


    class MotorNameNotKnownException extends Exception {

        MotorNameNotKnownException(String message) {
            super(message);
        }
    }

    class MotorStalledException extends Exception {

        MotorStalledException(String message) {
            super(message);
        }
    }


    /***********************************************
     *
     *     IMU ANGLES AND DEGREES
     *
     ***********************************************/


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public RelicRecoveryVuMark getPictograph(rr_OpMode aOpMode) throws InterruptedException {
        aOpMode.telemetry.setAutoClear(true); //neccessary for using Vuforia
        int cameraMonitorViewId = aOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", aOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AevlBL3/////AAAAGZ3T16bk1EepnUsSLPkQW/sFqYxxQLGZ0w6paGMug92slctEFAuXjXeMqrzDuCLvLZmY1sWjvn4kb5WKPKH4RdCZB7ccft3XGKh8rVn0r+TxhcJUmZwsdciAzCBYVe5FLnGtldKTV1eVbNFcN6FpDfZstRXXBdjqyMBg5XzJmhJp5rcG5TIi0qMcjaoHFqaBdnMyYBAeERylDVGBbDbIAX0dLDiQ5bjxA/lAphyHjDDyetpVjGlEwziUzcYbdvZK3zjGpR7WH62RqM6QzO1s7PcTppQMgRi3FxhisqKKZdWWF5pFGBPMP6bpsOzHTd8TDxPjwXiYIZxt3MwkhQ+1JpyAG9CVo+I0T/b/oNT0/ulZ";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        for (int i = 0; i < 25; i++) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                aOpMode.telemetryAddData("VuMark", "VuMark Left", "Left visible");
                aOpMode.telemetryUpdate();
                detectedPictograph = pictographType.LEFT;
                return vuMark;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                aOpMode.telemetryAddData("VuMark", "VuMark Center", "Center visible");
                aOpMode.telemetryUpdate();
                detectedPictograph = pictographType.CENTER;
                return vuMark;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                aOpMode.telemetryAddData("VuMark", "VuMark Right", "Right visible");
                aOpMode.telemetryUpdate();
                detectedPictograph = pictographType.RIGHT;

                return vuMark;
            } else {
                aOpMode.telemetryAddData("VuMark", "Unknown", "No VuMark Detected");
                aOpMode.telemetryUpdate();
                detectedPictograph = pictographType.UNKNOWN;
            }

            Thread.sleep(100);
            //aOpMode.telemetryUpdate();
        }

        return RelicRecoveryVuMark.UNKNOWN;

    }

    public void setTrayHeightPositionWithTouchLimits(rr_OpMode aOpMode, int position, float power) throws InterruptedException{

        //set the mode to be RUN_TO_POSITION
        motorArray[TRAY_LIFT_MOTOR].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Thread.sleep(50);

        //Now set the target
        motorArray[TRAY_LIFT_MOTOR].setTargetPosition(position);

        //now set the power
        motorArray[TRAY_LIFT_MOTOR].setPower(power);

        //reset clock for checking stall
        aOpMode.reset_timer_array(GENERIC_TIMER);


        while (motorArray[TRAY_LIFT_MOTOR].isBusy() &&
                (((position < motorArray[TRAY_LIFT_MOTOR].getCurrentPosition()) && !isTrayLowerLimitPressed())||
                ((position > motorArray[TRAY_LIFT_MOTOR].getCurrentPosition()) && !isTrayUpperLimitPressed()))
                && (aOpMode.time_elapsed_array(GENERIC_TIMER) < MAX_MOTOR_LOOP_TIME)
                &&(!aOpMode.gamepad1.a) //check for cancel.
                        && Math.abs(motorArray[TRAY_LIFT_MOTOR].getCurrentPosition() - position)
                        > rr_Constants.MOTOR_ENCODER_THRESHOLD) {
            ((TeleOpJ2) aOpMode).lib.processTeleOpDrive();
        }
        //stop the motor
        motorArray[TRAY_LIFT_MOTOR].setPower(0.0f);

        motorArray[TRAY_LIFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTrayFlipPosition(rr_OpMode aOpMode, float position) throws InterruptedException{
        trayFlipServo.setPosition(position);
        trayFlipPosition=position;
//        Thread.sleep(750);
    }

    public void setCubePusherPosition(rr_OpMode aOpMode, float position) throws InterruptedException{
        cubePusherServo.setPosition(position);
        cubePusherPosition=position;
        Thread.sleep(250);
    }

    public void setCubeHolderPosition(rr_OpMode aOpMode, float position) throws InterruptedException{
        cubeHolderServo.setPosition(position);
        cubeHolderPosition=position;
        Thread.sleep(250);
    }


    public int getTrayPosition(rr_OpMode aOpMode) throws InterruptedException{
        return motorArray[TRAY_LIFT_MOTOR].getCurrentPosition();
    }

    public void setIntakePower(rr_OpMode aOpMode, float leftPower, float rightPower){
        motorArray[INTAKE_RIGHT_MOTOR].setPower(rightPower);
        motorArray[INTAKE_LEFT_MOTOR].setPower(leftPower);
    }

    public double getIntakeOpticalRightSensorRange(rr_OpMode aOpMode) throws InterruptedException{
        return intakeRightRangeSensor.cmOptical();
    }

    public double getIntakeUltrasonicRightSensorRange(rr_OpMode aOpMode) throws InterruptedException{
        return intakeRightRangeSensor.cmUltrasonic();
    }

    public void initTrayPosition(rr_OpMode aOpMode) throws InterruptedException{
        motorArray[TRAY_LIFT_MOTOR].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aOpMode.reset_timer_array(GENERIC_TIMER);
        while(!isTrayLowerLimitPressed()&&!isTrayUpperLimitPressed() && aOpMode.time_elapsed_array(GENERIC_TIMER)<MAX_MOTOR_LOOP_TIME){
            setTrayLiftPower(aOpMode, -TRAY_LIFT_POWER/2);
        }
        setTrayLiftPower(aOpMode, 0);
        motorArray[TRAY_LIFT_MOTOR].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void rotateWheel(rr_OpMode aOpMode, int motorNumber,
                            rr_Constants.DirectionEnum direction)
            throws InterruptedException{
        aOpMode.reset_timer_array(GENERIC_TIMER);
        motorArray[motorNumber].setPower(0.5f);
        while(aOpMode.time_elapsed_array(GENERIC_TIMER)<2000){
            //do nothing
        }
        motorArray[motorNumber].setPower(0.0f);

    }

}
