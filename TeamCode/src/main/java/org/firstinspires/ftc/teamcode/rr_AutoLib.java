package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.InputMismatchException;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;


import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_HOLDER_RELEASE_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_PUSHED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.SidewaysRight;
import static org.firstinspires.ftc.teamcode.rr_Constants.GENERIC_TIMER;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_HIGH;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.rr_Constants.ROBOT_TRACK_DISTANCE;


public class rr_AutoLib {

    //TODO: Declarations of Conditions

    protected FalseCondition falseStop = new FalseCondition();
//    protected EopdProximityCondition eopdProximityStop = new EopdProximityCondition();
//    protected RangeSensorProximityOrColorVerifiedCondition rangeSensorProximityOrColorVerifiedStop =
//            new RangeSensorProximityOrColorVerifiedCondition();
//    protected RangeSensorOpticalProximityCondition rangeSensorOpticalProximityStop =
//            new RangeSensorOpticalProximityCondition();
//    protected RangeSensorUltraSonicProximityCondition rangeSensorUltraSonicProximityStop =
//            new RangeSensorUltraSonicProximityCondition();
//    protected RangeSensorUltraSonicCornerPositioningCondition rangeSensorUltraSonicCornerPositioningStop =
//            new RangeSensorUltraSonicCornerPositioningCondition();
//    protected colorPressVerifiedCondition colorPressVerifiedStop = new colorPressVerifiedCondition();

    rr_Robot robot;
    rr_OpMode aOpMode;
    float columnDistance;

    public rr_AutoLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode);
        this.aOpMode = aOpMode;
        robot.autonomousInit(aOpMode, aHwMap);
    }

    public void universalMoveRobot(rr_OpMode aOpMode, double polarAngle,
                                   double polarVelocity, double rotationalVelocity,
                                   long duration, rr_OpMode.StopCondition condition,
                                   boolean isPulsed, long pulseWidthDuration, long pulseRestDuration)
            throws InterruptedException {


        robot.universalMoveRobotWithCondition(aOpMode, polarVelocity * Math.sin(Math.toRadians(polarAngle)),
                polarVelocity * Math.cos(Math.toRadians(polarAngle)), rotationalVelocity, duration, condition, isPulsed, pulseWidthDuration, pulseRestDuration);
    }

    // robot starts closer to audience
    public void blueOneAutonomousCommonAction(rr_OpMode aOpMode) throws InterruptedException {

        robot.setJewelArmDownPush();
        Thread.sleep(250);

        aOpMode.DBG("In Blue One Common");

        // read Vuforia pictograph
        aOpMode.telemetry.setAutoClear(true); //necessary for using Vuforia

        switch (robot.getPictograph(aOpMode)) {
            case RIGHT:
                columnDistance = 34 + 7;
                break; // 7 inches between cube columns
            case CENTER:
                columnDistance = 34;
                break; // 7 inches between cube columns
            case LEFT:
                columnDistance = 34 - 7;
                break; // 7 inches between cube columns
            default:
                columnDistance = 34;
                break;
        }
        aOpMode.telemetry.setAutoClear(false); //turning off the auto clear afterward

        aOpMode.DBG("Detected" + robot.detectedPictograph);

        Thread.sleep(1000);

        // now score the jewels
        aOpMode.telemetry.setAutoClear(true);

        pushJewelOpenCV(aOpMode, rr_Constants.AllianceColorEnum.BLUE);
        Thread.sleep(300);

        // move to the correct column based on the detected pictograph
        moveWheels(aOpMode, columnDistance, .2f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 95);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 95);
        Thread.sleep(100);
        moveWheels(aOpMode, 6, .4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 95);
        Thread.sleep(100);

        // score cubes, move back, and then push to ensure scoring
        scoreCube(aOpMode);
        Thread.sleep(500);
        moveWheels(aOpMode, 6, .4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(100);
        moveWheels(aOpMode, 9, .4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);

        // ensure that glyph has been scored
        moveWheels(aOpMode, 8, 0.4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(100);
        moveWheels(aOpMode, 9, 0.4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);
        moveWheels(aOpMode, 9, 0.4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(100);
    }

    // robot starts farther away from audience
    public void blueTwoAutonomousCommonAction(rr_OpMode aOpMode) throws InterruptedException {

        robot.setJewelArmDownPush();
        Thread.sleep(250);

        aOpMode.DBG("In Blue Two Common");

        rr_Constants.DirectionEnum moveDirection;

        // read Vuforia pictograph
        aOpMode.telemetry.setAutoClear(true);

        switch (robot.getPictograph(aOpMode)) {
            case RIGHT:
                columnDistance = 15 + 7;
                moveDirection = rr_Constants.DirectionEnum.SidewaysRight;
                break;
            case CENTER:
                columnDistance = 15;
                moveDirection = rr_Constants.DirectionEnum.SidewaysRight;
                break;
            case LEFT:
                columnDistance = 15 - 7;
                moveDirection = rr_Constants.DirectionEnum.SidewaysLeft;
                break;
            default:
                columnDistance = 15;
                moveDirection = rr_Constants.DirectionEnum.SidewaysRight;
                break;
        }
        aOpMode.telemetry.setAutoClear(false); //turning off the auto clear afterward

        aOpMode.DBG("Detected" + robot.detectedPictograph);

        // now score the jewels
        aOpMode.telemetry.setAutoClear(true);

        pushJewelOpenCV(aOpMode, rr_Constants.AllianceColorEnum.BLUE);
        Thread.sleep(300);

        // move to the correct column based on the detected pictograph
        moveWheels(aOpMode, 27, .2f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 175);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 175);
        Thread.sleep(100);
        moveWheels(aOpMode, columnDistance, .4f, rr_Constants.DirectionEnum.SidewaysLeft, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 175);
        Thread.sleep(100);

        // score cubes, move back, and then push to ensure scoring
        scoreCube(aOpMode);
        Thread.sleep(500);
        moveWheels(aOpMode, 7, 0.4f, Forward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 10, 0.4f, Backward, false);
        Thread.sleep(100);

        // ensure that glyph has been scored
        moveWheels(aOpMode, 5, 0.4f, Forward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 7, 0.4f, Backward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 7, 0.4f, Forward, false);
        Thread.sleep(100);
    }


    // robot starts closer to audience
    public void redOneAutonomousCommonAction(rr_OpMode aOpMode) throws InterruptedException {

        robot.setJewelArmDownPush();
        Thread.sleep(250);

        rr_Constants.DirectionEnum moveDirection;

        aOpMode.DBG("In Red One Common");

        aOpMode.telemetry.setAutoClear(true); //necessary for using Vuforia

        switch (robot.getPictograph(aOpMode)) {
            case RIGHT: {
                columnDistance = 38 - 7;
                break; // 7 inches between cube columns
            }
            case CENTER:
                columnDistance = 38;
                break; // 7 inches between cube columns
            case LEFT:
                columnDistance = 38 + 7;
                break; // 7 inches between cube columns
            default:
                columnDistance = 38;
                break;
        }

        aOpMode.telemetry.setAutoClear(false); //turning off the auto clear afterward

        aOpMode.DBG("Detected" + robot.detectedPictograph);

        // now score the jewels
        aOpMode.telemetry.setAutoClear(true);

        pushJewelOpenCV(aOpMode, rr_Constants.AllianceColorEnum.RED);
        Thread.sleep(300);

        // move to the correct column based on the detected pictograph
        moveWheels(aOpMode, columnDistance, .2f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 95);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 95);
        Thread.sleep(100);
        moveWheels(aOpMode, 2, .4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 95);
        Thread.sleep(100);

        // score cubes, move back, and then push to ensure scoring
        scoreCube(aOpMode);
        Thread.sleep(500);
        moveWheels(aOpMode, 9, .4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(100);
        moveWheels(aOpMode, 11, .4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);

        // ensure that glyph has been scored
        moveWheels(aOpMode, 7, 0.4f, Forward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 8, 0.4f, Backward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 7, 0.4f, Forward, false);
        Thread.sleep(100);
    }


    // robot starts farther away from audience
    public void redTwoAutonomousCommonAction(rr_OpMode aOpMode) throws InterruptedException {

        rr_Constants.DirectionEnum moveDirection;

        robot.setJewelArmDownPush();
        Thread.sleep(250);

        aOpMode.telemetry.setAutoClear(true);

        switch (robot.getPictograph(aOpMode)) {
            case RIGHT:
                columnDistance = 15 - 7;
                break; // 7 inches between cube columns
            case CENTER:
                columnDistance = 15;
                break; // 7 inches between cube columns
            case LEFT:
                columnDistance = 15 + 7;
                break; // 7 inches between cube columns
            default:
                columnDistance = 15;
                break;
        }
        aOpMode.telemetry.setAutoClear(false); //turning off the auto clear afterward

        aOpMode.DBG("Detected" + robot.detectedPictograph);

        // now score the jewels
        aOpMode.telemetry.setAutoClear(true);
        pushJewelOpenCV(aOpMode, rr_Constants.AllianceColorEnum.RED);
        Thread.sleep(300);

        // move to the correct column based on the detected pictograph
        moveWheels(aOpMode, 2, .3f, rr_Constants.DirectionEnum.SidewaysRight, true);
        Thread.sleep(100);
        moveWheels(aOpMode, 29, .3f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 5);
        Thread.sleep(100);
        moveWheels(aOpMode, columnDistance, .4f, rr_Constants.DirectionEnum.SidewaysRight, true);
        Thread.sleep(100);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 5);
        Thread.sleep(100);

        // score cubes, move back, and then push to ensure scoring
        scoreCube(aOpMode);
        Thread.sleep(500);
        moveWheels(aOpMode, 7, 0.4f, Forward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 11, 0.4f, Backward, false);
        Thread.sleep(100);

        // ensure that glyph has been scored
        moveWheels(aOpMode, 5, 0.4f, Forward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 9, 0.4f, Backward, false);
        Thread.sleep(100);
        moveWheels(aOpMode, 7, 0.4f, Forward, false);
        Thread.sleep(100);
    }

    public void adjustJewelArmUsingRange(rr_OpMode aOpMode) throws InterruptedException {


        // FOR ROBOT WITH VERTICAL RANGE SENSOR
        /*
        // TODO: change original jewel arm position
        robot.setJewelArmPosition(1.0f);

        if (robot.getFilteredLeftJewelRangeReading(aOpMode) > rr_Constants.JEWEL_DISTANCE ) {
            // while loop to lower jewel arm until specified distance from jewel
            while (robot.getFilteredLeftJewelRangeReading(aOpMode) > rr_Constants.JEWEL_DISTANCE) {
                // TODO: change increment going down
                robot.setJewelArmPosition(robot.getJewelArmPosition() - .01f);
            }
        }
        else {
            while(robot.getFilteredLeftJewelRangeReading(aOpMode) < rr_Constants.JEWEL_DISTANCE) {
                // TODO: change increment going up
                robot.setJewelArmPosition(robot.getJewelArmPosition() + .01f);
            }
        }
  */

        // FOR ROBOT WITH HORIZONTAL RANGE SENSOR
        // TODO: change value
        float minDistanceFromJewel = 1000f;
        float servoPosition = 0.0f;
        // TODO: change distance above jewel
        robot.setJewelArmPosition(aOpMode, .85f);

        // Scan the range from jewel down to a certain position and store minimum distance


        while (robot.getJewelArmPosition() < rr_Constants.BOTTOM_JEWEL_POSITION) {
            robot.setJewelArmPosition(aOpMode, robot.getJewelArmPosition() + (float) rr_Constants.JEWEL_ARM_INCREMENT);

            if (minDistanceFromJewel > robot.getFilteredLeftJewelRangeReading(aOpMode)) {
                minDistanceFromJewel = (float) robot.getFilteredLeftJewelRangeReading(aOpMode);
                aOpMode.telemetryAddData("minDistance  ", "minDistance", "minDistance" + minDistanceFromJewel);
                servoPosition = (float) robot.getJewelArmPosition();
            }
        }


        robot.setJewelArmPosition(aOpMode, servoPosition);

    }


    public void pushJewelOpenCV(rr_OpMode aOpMode, rr_Constants.AllianceColorEnum teamColor) throws InterruptedException {

        JewelDetector jewelDetector = new JewelDetector();
        jewelDetector.init(aOpMode.hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.enable();

        // This works - not sure why - do not change
        for (int i = 0; i < 100; i++) {
            jewelDetector.getCurrentOrder();
            Thread.sleep(10);
        }

        aOpMode.telemetry.addLine(String.valueOf(jewelDetector.getCurrentOrder()));
        aOpMode.telemetry.update();
        Thread.sleep(1000);

        if (teamColor == rr_Constants.AllianceColorEnum.BLUE) {

            if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.BLUE_RED) {
                aOpMode.telemetryAddData("Jewel Order", "BLUE_RED", "Left is Blue");
                robot.setJewelArmDownPush();
                Thread.sleep(250);
                //robot.turnAbsoluteBoschGyroDegreesAuto(aOpMode, 15);
                moveWheels(aOpMode, 2.5f, .2f, rr_Constants.DirectionEnum.Forward, true);
                Thread.sleep(250);
                robot.setJewelArmUp();
                Thread.sleep(250);
                columnDistance -= 2.5f; //accounting for forward motion already done.

            } else if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.RED_BLUE) {
                aOpMode.telemetryAddData("Jewel Order", "RED_BLUE", "Right is Blue");
                robot.setJewelArmDownPush();
                Thread.sleep(250);
                // robot.turnAbsoluteBoschGyroDegreesAuto(aOpMode, -15);
                moveWheels(aOpMode, 2.5f, .2f, rr_Constants.DirectionEnum.Backward, true);
                Thread.sleep(250);
                robot.setJewelArmUp();
                Thread.sleep(250);
                columnDistance += 2.5f; //accounting for backward motion already done.

            } else if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.UNKNOWN) {
                robot.setJewelArmDownPush();
                aOpMode.telemetryAddData("No Color Detected", "UNKNOWN", "Unknown");
            }
        }

        if (teamColor == rr_Constants.AllianceColorEnum.RED) {
            if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.BLUE_RED) {
                aOpMode.telemetryAddData("Jewel Order", "BLUE_RED", "Right is Red");
                robot.setJewelArmDownPush();
                Thread.sleep(250);
                //robot.turnAbsoluteBoschGyroDegreesAuto(aOpMode, -15);
                moveWheels(aOpMode, 2.5f, .2f, rr_Constants.DirectionEnum.Backward, true);
                Thread.sleep(250);
                robot.setJewelArmUp();
                Thread.sleep(250);
                columnDistance -= 2.5f; //accounting for forward motion already done.

            } else if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.RED_BLUE) {
                aOpMode.telemetryAddData("Jewel Order", "RED_BLUE", "Left is Red");
                robot.setJewelArmDownPush();
                Thread.sleep(250);
                //robot.turnAbsoluteBoschGyroDegreesAuto(aOpMode, 15);
                moveWheels(aOpMode, 2.5f, .2f, rr_Constants.DirectionEnum.Forward, true);
                Thread.sleep(250);
                robot.setJewelArmUp();
                Thread.sleep(250);
                columnDistance += 2.5f; //accounting for backward motion already done.
            } else if (jewelDetector.getCurrentOrder() == JewelDetector.JewelOrder.UNKNOWN) {
                aOpMode.telemetryAddData("No Color Detected", "UNKNOWN", "Unknown");
            }
        }
        aOpMode.telemetryUpdate();
        Thread.sleep(500);

        robot.setJewelArmUp();

        jewelDetector.disable();
    }


    public void detectColorAndPushJewel(rr_OpMode aOpMode, rr_Constants.AllianceColorEnum teamColor) throws InterruptedException {

        robot.setJewelArmDownPush();

        Thread.sleep(500);

        rr_Constants.JewelColorEnum leftJewelColor = robot.getJewelLeftColor(aOpMode);
        rr_Constants.JewelColorEnum rightJewelColor = robot.getJewelRightColor(aOpMode);

        aOpMode.DBG("In detectColorAndPushJewel");


        if (teamColor == rr_Constants.AllianceColorEnum.BLUE) {
            aOpMode.DBG("In detectColorAndPushJewel Blue Alliance");

            if ((leftJewelColor == rr_Constants.JewelColorEnum.BLUE) &&
                    (rightJewelColor == rr_Constants.JewelColorEnum.RED)) {
                aOpMode.telemetryAddData("LEFT IS BLUE", "LEFT_BLUE", "Left is Blue");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushRightJewel();
                if (leftJewelColor == rr_Constants.JewelColorEnum.RED) {
                    robot.pushLeftJewel();
                }
            } else if ((leftJewelColor == rr_Constants.JewelColorEnum.RED) &&
                    (rightJewelColor == rr_Constants.JewelColorEnum.BLUE)) {
                aOpMode.telemetryAddData("RIGHT IS BLUE", "RIGHT_BLUE", "RIGHT is Blue");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushLeftJewel();
                if (rightJewelColor == rr_Constants.JewelColorEnum.RED) {
                    robot.pushRightJewel();
                }
            } else if (((leftJewelColor) == rr_Constants.JewelColorEnum.UNKNOWN) ||
                    (rightJewelColor == rr_Constants.JewelColorEnum.UNKNOWN)) {
                aOpMode.telemetryAddData("Unknown:BlueTeam", "Unknown", "No color detected");
                aOpMode.telemetryAddData("Left color", "left color", "Left color" + robot.getJewelLeftColor(aOpMode));
                aOpMode.telemetryAddData("Right color", "Right color", "Right color" + robot.getJewelRightColor(aOpMode));
                aOpMode.telemetryUpdate();
            }

        }

        aOpMode.DBG("Exiting blue alliance detect color and push");

        if (teamColor == rr_Constants.AllianceColorEnum.RED) {
            aOpMode.DBG("In detectColorAndPushJewel Red Alliance");

            if ((leftJewelColor == rr_Constants.JewelColorEnum.BLUE) &&
                    (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.RED)) {
                aOpMode.telemetryAddData("RIGHT IS RED", "RIGHT_RED", "Right is RED");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushLeftJewel();
                if (rightJewelColor == rr_Constants.JewelColorEnum.BLUE) {
                    robot.pushRightJewel();
                }
            } else if ((leftJewelColor == rr_Constants.JewelColorEnum.RED) &&
                    (rightJewelColor == rr_Constants.JewelColorEnum.BLUE)) {
                aOpMode.telemetryAddData("LEFT IS RED", "LEFT_RED", "Left is RED");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushRightJewel();
                if (leftJewelColor == rr_Constants.JewelColorEnum.BLUE) {
                    robot.pushLeftJewel();
                }
            } else if ((leftJewelColor == rr_Constants.JewelColorEnum.UNKNOWN) ||
                    ((rightJewelColor) == rr_Constants.JewelColorEnum.UNKNOWN)) {
                aOpMode.telemetryAddData("Unknown: RedTeam", "Unknown", "No color detected");
                aOpMode.telemetryAddData("Left color", "left color", "Left color" + robot.getJewelLeftColor(aOpMode));
                aOpMode.telemetryAddData("Right color", "Right color", "Right color" + robot.getJewelRightColor(aOpMode));
                aOpMode.telemetryUpdate();
            }
        }

        robot.setJewelPusherNeutral();
        Thread.sleep(500);
        robot.setJewelArmUp();
        aOpMode.DBG("Exiting detect color and push");

    }


    /**
     * moveWheels method
     *
     * @param aOpMode   - object of vv_OpMode class
     * @param distance  - in inches
     * @param power     - float
     * @param Direction - forward, backward, sideways left, or sideways right
     * @throws InterruptedException
     */
    public void moveWheels(rr_OpMode aOpMode, float distance, float power,
                           rr_Constants.DirectionEnum Direction, boolean isRampedPower)
            throws InterruptedException {
        if (Direction == Forward) {
            moveRobotToPositionFB(aOpMode, distance * (2f/3), power, isRampedPower);
        } else if (Direction == Backward) {
            moveRobotToPositionFB(aOpMode, -distance * (2f/3), power, isRampedPower);
        } else if (Direction == rr_Constants.DirectionEnum.SidewaysLeft) {
            moveRobotToPositionSideways(aOpMode, distance * (2f/3), power, isRampedPower);
        } else if (Direction == rr_Constants.DirectionEnum.SidewaysRight) {
            moveRobotToPositionSideways(aOpMode, -distance * (2f/3), power, isRampedPower);
        }
    }

    /**
     * Runs robot to a specific position while driving forwards or backwards
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
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) (((distance) / (Math.PI * MECANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //using the generic method with all powers set to the same value and all positions set to the same position
        robot.runRobotToPosition(aOpMode, power, power, power, power,
                targetPosition, targetPosition, targetPosition, targetPosition, isRampedPower);
    }


    /**
     * Runs robot to a specific position while driving sideways
     *
     * @param aOpMode  an object of the rr_OpMode class
     * @param distance distance wheels will go in inches
     * @param power    generic power of the motors (positive = left, negative = right)
     */
    public void moveRobotToPositionSideways(rr_OpMode aOpMode, float distance,
                                            float power, boolean isRampedPower)
            throws InterruptedException {
        //we need to
        //store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * MECANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
        //using the generic method with all powers set to the same value and all positions set to the same position
        robot.runRobotToPosition(aOpMode, power, power, power, power,
                -targetPosition, targetPosition, targetPosition, -targetPosition, isRampedPower);
    }

    public class FalseCondition implements rr_OpMode.StopCondition {
        //can be used as an empty condition, so the robot keeps running in universal movement
        public boolean stopCondition(rr_OpMode aOpMode) throws InterruptedException {
            return (false);
        }
    }

    public void scoreCube(rr_OpMode aOpMode) throws InterruptedException {
        robot.setCubeHolderPosition(aOpMode, rr_Constants.CUBE_HOLDER_RELEASE_POSITION);
        Thread.sleep(250);
        alignCubes(aOpMode);
        robot.setTrayHeightPositionWithTouchLimits(aOpMode, rr_Constants.TRAY_HEIGHT_COLLECTION_POSITION, rr_Constants.TRAY_LIFT_POWER);
        Thread.sleep(750);
        robot.setTrayFlipPosition(aOpMode, rr_Constants.TRAY_FLIP_SCORING_POSITION);
        Thread.sleep(750);
        robot.setTrayFlipPosition(aOpMode, rr_Constants.TRAY_FLIP_COLLECTION_POSITION);
        Thread.sleep(750);
    }

    public void alignCubes(rr_OpMode aOpMode) throws InterruptedException{
        robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_RESTED_POSITION);
        Thread.sleep(500);
        robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_PUSHED_POSITION);
        Thread.sleep(500);
        robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_RESTED_POSITION);
    }


}

