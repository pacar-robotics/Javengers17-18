package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.text.DecimalFormat;

import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_MAX_DURATION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_ONE_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_TWO_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_VERTICAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.GENERIC_TIMER;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_ARM_DOWN_READ;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_ARM_UP;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_PUSHER_LEFT;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_PUSHER_NEUTRAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.JEWEL_PUSHER_RIGHT;
import static org.firstinspires.ftc.teamcode.rr_Constants.LEFT_MOTOR_TRIM_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.MAX_MOTOR_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.rr_Constants.MAX_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_ENCODER_MARGIN;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_FRONT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_SIDE_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.rr_Constants.MIN_ROBOT_TURN_MOTOR_VELOCITY;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_FB_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_FB_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_SIDEWAYS_POWER_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_RAMP_SIDEWAYS_POWER_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_EXTEND;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_ANGLE_EXTEND;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_ANGLE_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_LOWER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_MAX_DURATION;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RIGHT_MOTOR_TRIM_FACTOR;



public class rr_Robot {

    //NAVX Constants
    private final int NAVX_DIM_I2C_PORT = 2; //TODO: Change
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private final double TARGET_ANGLE_DEGREES = 90.0;

    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

    private final int DEVICE_TIMEOUT_MS = 500;

    HardwareMap hwMap = null;

    //Motors
    private DcMotor motorArray[];

    //Servo
    private Servo cubeClaw;
    private Servo cubeOrientation;
    private Servo jewelArm;
    private Servo jewelPusher;
    private Servo relicClaw;
    private Servo relicClawAngle;
    private Servo relicArmRetract; //Continuous servo

    //TODO: Color Sensors
    private ColorSensor leftJewelColorSensor;
    private ColorSensor rightJewelColorSensor;
    private ColorSensor frontFloorColorSensor;
    private ColorSensor backFloorColorSensor;

    protected navXPIDController yawPIDController;

    //Sensors
    private AHRS baseMxpGyroSensor; //NavX MXP gyro
    private ModernRoboticsI2cRangeSensor rangeSensor;

    //Variables for Ramped Power
    private double prevFLVelocity = 0.0f;
    private double prevFRVelocity = 0.0f;
    private double prevBLVelocity = 0.0f;
    private double prevBRVelocity = 0.0f;

    private ElapsedTime period = new ElapsedTime();

    public void init(rr_OpMode aOpMode, HardwareMap ahwMap) throws InterruptedException {
        // save reference to HW Map
        aOpMode.DBG("in Robot init");
        hwMap = ahwMap;

        //Define and Initialize motors, sensors, and servos

        //Instantiate motorArray
        motorArray = new DcMotor[10];

        //Map Motors
        motorArray[FRONT_LEFT_MOTOR] = hwMap.get(DcMotor.class, "motor_front_left");
        motorArray[FRONT_RIGHT_MOTOR] = hwMap.get(DcMotor.class, "motor_front_right");
        motorArray[BACK_LEFT_MOTOR] = hwMap.get(DcMotor.class, "motor_back_left");
        motorArray[BACK_RIGHT_MOTOR] = hwMap.get(DcMotor.class, "motor_back_right");
//        motorArray[CUBE_ARM] = hwMap.get(DcMotor.class, "motor_cube_arm");
//        motorArray[RELIC_ARM_EXTEND] = hwMap.get(DcMotor.class, "motor_relic_extend");


        //TODO: Map Sensors and Servos

        //Map Servos
//        cubeClaw = hwMap.get(Servo.class, "servo_cube_claw");
//        cubeOrientation = hwMap.get(Servo.class, "servo_cube_orientatoin");
        jewelArm = hwMap.get(Servo.class, "servo_jewel_arm");
        jewelPusher = hwMap.get(Servo.class, "servo_jewel_pusher");
//        relicClaw = hwMap.get(Servo.class, "servo_relic_claw");
//        relicClawAngle = hwMap.get(Servo.class, "servo_claw_angle");
//        relicArmRetract = hwMap.get(Servo.class, "servo_relic_retract");

        //Map Sensors
        leftJewelColorSensor = hwMap.get(ColorSensor.class, "left_jewel_color");
        rightJewelColorSensor = hwMap.get(ColorSensor.class, "right_jewel_color");
//        frontFloorColorSensor = hwMap.get(ColorSensor.class, "front_floor_color_sensor");
//        backFloorColorSensor = hwMap.get(ColorSensor.class, "back_floor_color_sensor");
//        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");


//        aOpMode.DBG("Begin Gyro Calib");
//        //allocate the mxp gyro sensor.
//        baseMxpGyroSensor = AHRS.getInstance(hwMap.deviceInterfaceModule.get("dim"),
//                NAVX_DIM_I2C_PORT,
//                AHRS.DeviceDataType.kProcessedData);
//
//        while (baseMxpGyroSensor.isCalibrating()) {
//            aOpMode.idle();
//            Thread.sleep(50);
//            aOpMode.telemetryAddData("1 navX-Device", "Status:",
//                    baseMxpGyroSensor.isCalibrating() ?
//                            "CALIBRATING" : "Calibration Complete");
//            aOpMode.telemetryUpdate();
//        }
//
//        aOpMode.DBG("Gyro Calib Complete");


        //zero out the yaw value, so this will be the frame of reference for future calls.
        //do not call this for duration of run after this.
        //baseMxpGyroSensor.zeroYaw();

        //wait for these servos to reach desired state


        aOpMode.DBG("Starting Motor Setups");

        //Set the Direction of Motors
        motorArray[FRONT_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[FRONT_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[BACK_LEFT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);
        motorArray[BACK_RIGHT_MOTOR].setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all base motors to zero power
        stopBaseMotors(aOpMode);

        aOpMode.DBG("Presetting Servos");

        //Setting servos to intitial position TODO: CHANGE
//        closeCubeClawServoOneCube();
//        setCubeClawToHorizontal();
        setJewelPusherNeutral();
        setJewelArmUp();
//        setRelicClawClosed();
//        setRelicArmAngleGrab();
        Thread.sleep(1000);

        aOpMode.DBG("Exiting Robot init");
    }


    //MOTOR METHODS


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


    //SENSOR METHODS


    public int getRangeSensorUltrasonicRaw(rr_OpMode aOpMode) {
        return rangeSensor.rawUltrasonic();
    }

    public int getRangeSensorOpticalRaw(rr_OpMode aOpMode) {
        return rangeSensor.rawOptical();
    }

    public double getRangeSensorDistance(rr_OpMode aOpMode) {
        return rangeSensor.getDistance(DistanceUnit.CM);
    }

    public double getRangeSensorOpticalCM(rr_OpMode aOpMode) {
        return rangeSensor.cmOptical();
    }

    public float getMxpGyroSensorHeading(rr_OpMode aOpMode) {
        return baseMxpGyroSensor.getYaw();
    }

    protected void setMxpGyroZeroYaw(rr_OpMode aOpMode) {
        baseMxpGyroSensor.zeroYaw();
    }

    public double getFloorBlueReading() {
        return frontFloorColorSensor.blue();
    }

    public double getFloorRedReading() {
        return backFloorColorSensor.red();
    }


    //CONTROL OF WHEELS


    /**
     * Runs robot to a specific position. Can be called by other, more specific methods to move forwards, backwards or sideways.
     *
     * @param aOpMode     an object of the rr_OpMode class
     * @param fl_Power    front right motor power
     * @param fr_Power    front left motor power
     * @param bl_Power    back left motor power
     * @param br_Power    back right motor power
     * @param fl_Position front left motor position
     * @param fr_Position front left motor position
     * @param bl_Position back left motor position
     * @param br_Position back right motor position
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

            if (DEBUG) {
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
        while (motorArray[BACK_RIGHT_MOTOR].getPower() != 0) {
            aOpMode.idle();
        }
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

        //blend with prev velocities to smooth out start

        fl_velocity = ((yAxisVelocity + xAxisVelocity) + prevFLVelocity) / 2;

        fr_velocity = ((yAxisVelocity - xAxisVelocity) + prevFRVelocity) / 2;

        bl_velocity = ((yAxisVelocity - xAxisVelocity) + prevBLVelocity) / 2;

        br_velocity = ((yAxisVelocity + xAxisVelocity) + prevBRVelocity) / 2;

        //save these in variables that are part of vvRobot to be used in next cycle.

        prevFLVelocity = fl_velocity;
        prevFRVelocity = fr_velocity;
        prevBLVelocity = bl_velocity;
        prevBRVelocity = br_velocity;


        motorArray[FRONT_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[FRONT_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[BACK_LEFT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArray[BACK_RIGHT_MOTOR].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(50);

        //apply specific powers to motors to get desired movement
        //wait till duration is complete.


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

    public void turnPidMxpAbsoluteDegrees(rr_OpMode aOpMode, float turndegrees, float toleranceDegrees)
            throws InterruptedException {
         /* Create a PID Controller which uses the Yaw Angle as input. */
        //by default the PIDController is disabled. turn it on.

        // the mxp gyro sensor classes include a built in
        // proportional Integrated Derivative (PID) adjusted control loop function
        //lets set that up for reading the YAW value (rotation around the z axis for the robot).

        yawPIDController = new navXPIDController(baseMxpGyroSensor,
                navXPIDController.navXTimestampedDataSource.YAW);

        //Configure the PID controller for the turn degrees we want
        yawPIDController.setSetpoint(turndegrees);
        yawPIDController.setContinuous(true);

        //limits of motor values (-1.0 to 1.0)
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        //tolerance degrees is defined to prevent oscillation at high accuracy levels.
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, toleranceDegrees);
        //PID initial parameters, usually found by trial and error.

        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        yawPIDController.enable(true);


        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        aOpMode.reset_timer_array(GENERIC_TIMER);

        while ((aOpMode.time_elapsed_array(GENERIC_TIMER) < MAX_MOTOR_LOOP_TIME) &&
                !Thread.currentThread().isInterrupted()) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()) {
                    //we have reached turn target within tolerance requested.
                    //stop
                    aOpMode.telemetryAddData("On Target", ":Value:",
                            df.format(getMxpGyroSensorHeading(aOpMode)));
                    aOpMode.telemetryUpdate();
                    break;
                } else {
                    //get the new adjustment for direction from the PID Controller
                    float output = (float) yawPIDResult.getOutput();
                    //apply it to the motors, using one of our functions.
                    //if output was positive, the method below would turn the robot clockwise
                    runMotorsUsingEncoders(aOpMode, output, -output, output, -output);
                    aOpMode.telemetryAddData("PIDOutput", ":Value:", df.format(output) + ", " +
                            df.format(-output));
                    aOpMode.telemetryUpdate();
                }
            } else {
                /* A timeout occurred */
                Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
            aOpMode.idle();
            aOpMode.telemetryAddData("Yaw", ":Value:", df.format(getMxpGyroSensorHeading(aOpMode)));
            aOpMode.idle();
        }

    }


    //CUBE ARM CONTROL


    //used in TeleOp
    public void setCubeArmPower(rr_OpMode aOpMode, float power) {
        motorArray[CUBE_ARM].setPower(power * CUBE_ARM_POWER_FACTOR);
    }

    public void moveCubeArmToPositionWithLimits(rr_OpMode aOpMode, int position, float power) throws InterruptedException {
        //set the mode to be RUN_TO_POSITION
        motorArray[CUBE_ARM].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Thread.sleep(50);

        //Now set the target
        motorArray[CUBE_ARM].setTargetPosition(position);

        //now set the power
        motorArray[CUBE_ARM].setPower(power);

        //reset clock for checking stall
        aOpMode.reset_timer_array(GENERIC_TIMER);


        while (motorArray[CUBE_ARM].isBusy() && motorArray[CUBE_ARM].getCurrentPosition() < CUBE_ARM_UPPER_LIMIT &&
                motorArray[CUBE_ARM].getCurrentPosition() > CUBE_ARM_LOWER_LIMIT && (aOpMode.time_elapsed_array(GENERIC_TIMER) < CUBE_ARM_MAX_DURATION)) {
            aOpMode.idle();
        }
        //stop the motor
        motorArray[CUBE_ARM].setPower(0.0f);
    }

    public void openCubeClawServo() throws InterruptedException {
        cubeClaw.setPosition(CUBE_CLAW_OPEN);
        Thread.sleep(100);
    }

    public void closeCubeClawServoOneCube() throws InterruptedException {
        cubeClaw.setPosition(CUBE_CLAW_ONE_CLOSED);
        Thread.sleep(100);
    }

    public void closeCubeClawServoTwoCube() throws InterruptedException {
        cubeClaw.setPosition(CUBE_CLAW_TWO_CLOSED);
        Thread.sleep(100);
    }

    public void setCubeClawToHorizontal() throws InterruptedException {
        cubeOrientation.setPosition(CUBE_ORIENTATION_HORIZONTAL);
        Thread.sleep(100);
    }

    public void setCubeClawToVertical() throws InterruptedException {
        cubeOrientation.setPosition(CUBE_ORIENTATION_VERTICAL);
        Thread.sleep(100);
    }


    //RELIC ARM CONTROL


    public void extendRelicArmToPositionWithLimits(rr_OpMode aOpMode, int position, float power) throws InterruptedException {
        if (motorArray[RELIC_ARM_EXTEND].getCurrentPosition() < position) {

            //set the mode to be RUN_TO_POSITION
            motorArray[RELIC_ARM_EXTEND].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Thread.sleep(50);

            //Now set the target
            motorArray[RELIC_ARM_EXTEND].setTargetPosition(position);

            //now set the power
            motorArray[RELIC_ARM_EXTEND].setPower(power);

            //reset clock for stall
            aOpMode.reset_timer_array(GENERIC_TIMER);


            while (motorArray[RELIC_ARM_EXTEND].isBusy() && motorArray[RELIC_ARM_EXTEND].getCurrentPosition() < RELIC_UPPER_LIMIT &&
                    motorArray[RELIC_ARM_EXTEND].getCurrentPosition() > RELIC_LOWER_LIMIT && (aOpMode.time_elapsed_array(GENERIC_TIMER) < RELIC_MAX_DURATION)) {
                aOpMode.idle();
            }
            //stop the motor
            motorArray[CUBE_ARM].setPower(0.0f);
        } else {
            aOpMode.telemetryAddData("Error", "Relic Arm", "Asked to be retracted in extend method");
        }
    }

    public void setPowerExtendRelicArm(rr_OpMode aOpMode, float power) {
        motorArray[RELIC_ARM_EXTEND].setPower(power);
    }

    public void setPowerRetractRelicArm(rr_OpMode aOpMode, float power) {
        relicArmRetract.setPosition(power);
    }

    public void setRelicArmAngleGrab() throws InterruptedException {
        relicClawAngle.setPosition(RELIC_CLAW_ANGLE_GRAB);
        Thread.sleep(100);
    }

    public void setRelicArmAngleExtend() throws InterruptedException {
        relicClawAngle.setPosition(RELIC_CLAW_ANGLE_EXTEND);
        Thread.sleep(100);
    }

    public void setRelicClawClosed() throws InterruptedException {
        relicClaw.setPosition(RELIC_CLAW_CLOSED);
        Thread.sleep(100);
    }

    public void setRelicClawOpen() throws InterruptedException {
        relicClaw.setPosition(RELIC_CLAW_OPEN);
        Thread.sleep(100);
    }


    //JEWEL ARM CONTROL


    public void pushRightJewel() throws InterruptedException {
        jewelPusher.setPosition(JEWEL_PUSHER_RIGHT);
        Thread.sleep(100);
    }

    public void pushLeftJewel() throws InterruptedException {
        jewelPusher.setPosition(JEWEL_PUSHER_LEFT);
        Thread.sleep(100);
    }

    public void setJewelPusherNeutral() throws InterruptedException {
        jewelPusher.setPosition(JEWEL_PUSHER_NEUTRAL);
        Thread.sleep(100);
    }

    public void setJewelArmUp() throws InterruptedException {
        jewelArm.setPosition(JEWEL_ARM_UP);
        Thread.sleep(100);
    }

    public void moveArmToPushJewel() throws InterruptedException {
        jewelArm.setPosition(rr_Constants.JEWEL_ARM_DOWN_PUSH);
        Thread.sleep(100);
    }

    public void setJewelArmDown() throws InterruptedException {
        //jewelArm.setPosition(rr_Constants.JEWEL_ARM_DOWN);
        for(float armPosition = rr_Constants.JEWEL_ARM_UP; armPosition >= rr_Constants.JEWEL_ARM_DOWN_READ; armPosition -= .1f) {
            jewelArm.setPosition(armPosition);
            Thread.sleep(300);
        }
    }

    //JEWEL COLOR SENSORS


//    public rr_Constants.JewelColorEnum getJewelLeftColor(rr_OpMode aOpMode) throws InterruptedException {
//        Thread.sleep(500);
//
//        if((leftJewelColorSensor.red() - leftJewelColorSensor.blue()) > rr_Constants.JEWEL_COLOR_MARGIN) {
//            return rr_Constants.JewelColorEnum.RED;
//        }
//        if((leftJewelColorSensor.blue() - leftJewelColorSensor.red()) > rr_Constants.JEWEL_COLOR_MARGIN) {
//            return rr_Constants.JewelColorEnum.BLUE;
//        }
//
//        return rr_Constants.JewelColorEnum.UNKNOWN;
//    }
//
//    public rr_Constants.JewelColorEnum getJewelRightColor(rr_OpMode aOpMode) throws InterruptedException {
//        Thread.sleep(500);
//
//        if((leftJewelColorSensor.red() - leftJewelColorSensor.blue()) > rr_Constants.JEWEL_COLOR_MARGIN) {
//            return rr_Constants.JewelColorEnum.RED;
//        }
//        if((leftJewelColorSensor.blue() - leftJewelColorSensor.red()) > rr_Constants.JEWEL_COLOR_MARGIN) {
//            return rr_Constants.JewelColorEnum.BLUE;
//        }
//
//        return rr_Constants.JewelColorEnum.UNKNOWN;
//    }

    public rr_Constants.JewelColorEnum getJewelLeftColor(rr_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(500);

        if(leftJewelColorSensor.red() > leftJewelColorSensor.blue()) {
            return rr_Constants.JewelColorEnum.RED;
        }
        if(leftJewelColorSensor.blue() > leftJewelColorSensor.red()) {
            return rr_Constants.JewelColorEnum.BLUE;
        }

        return rr_Constants.JewelColorEnum.UNKNOWN;
    }

    public rr_Constants.JewelColorEnum getJewelRightColor(rr_OpMode aOpMode) throws InterruptedException {
        Thread.sleep(500);

        if(leftJewelColorSensor.red() > leftJewelColorSensor.blue()) {
            return rr_Constants.JewelColorEnum.RED;
        }
        if(leftJewelColorSensor.blue() > leftJewelColorSensor.red()) {
            return rr_Constants.JewelColorEnum.BLUE;
        }

        return rr_Constants.JewelColorEnum.UNKNOWN;
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

    public void setJewelArmPosition(float position) throws InterruptedException {
        jewelArm.setPosition(position);
        Thread.sleep(250);
    }

    public float getJewelArmPosition() {
        return (float) jewelArm.getPosition();
    }

    public void setJewelKnockerPosition(float position) throws InterruptedException {
        jewelPusher.setPosition(position);
        Thread.sleep(250);
    }

    public float getJewelKnockerPosition() {
        return (float) jewelPusher.getPosition();
    }


}