package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_LOWER_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_RAISE_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.rr_Constants.ROBOT_TRACK_DISTANCE;


public class rr_AutoLib {

    //TODO: Declarations of Conditions

    protected blueLineDetectCondition blueLineDetectStop = new blueLineDetectCondition();
    protected redLineDetectCondition redLineDetectStop = new redLineDetectCondition();
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

    public rr_AutoLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode, aHwMap);
        this.aOpMode = aOpMode;
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

        aOpMode.DBG("In Blue One Common");

        float columnDistance;

        robot.closeCubeClawServoOneCube();
        robot.moveCubeArmToPositionWithTouchLimits(aOpMode, rr_Constants.CUBE_ARM_MIDDLE, CUBE_ARM_RAISE_POWER);

        detectColorAndPushJewel(aOpMode, rr_Constants.AllianceColorEnum.BLUE);
        Thread.sleep(300);

        switch(robot.getPictograph(aOpMode)) {
            case RIGHT: columnDistance = 35; break; // 7 inches between cube columns
            case CENTER: columnDistance = 35 - 7; break; // 7 inches between cube columns
            case LEFT: columnDistance = 35 - 14; break; // 7 inches between cube columns
            default: columnDistance = 35 - 7; break;
        }

        moveWheels(aOpMode, columnDistance, .4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(250);
        moveWheels(aOpMode, 6, .4f, rr_Constants.DirectionEnum.SidewaysRight, true);
        Thread.sleep(250);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, -90);
        Thread.sleep(250);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, -90);
        Thread.sleep(250);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, -70);
        Thread.sleep(250);
        robot.moveCubeArmToPositionWithLimits(aOpMode, CUBE_ARM_GRAB, CUBE_ARM_LOWER_POWER);
        Thread.sleep(250);
        universalMoveRobot(aOpMode, 20, .3f, 0, 1500, falseStop, false, 0, 0);
        Thread.sleep(250);

    }

    // robot starts farther away from audience
    public void blueTwoAutonomousCommonAction(rr_OpMode aOpMode) throws InterruptedException {

        aOpMode.DBG("In Blue Two Common");

        float columnDistance;
        rr_Constants.DirectionEnum moveDirection;

        robot.closeCubeClawServoOneCube();
        robot.moveCubeArmToPositionWithTouchLimits(aOpMode, rr_Constants.CUBE_ARM_MIDDLE, CUBE_ARM_RAISE_POWER);

        detectColorAndPushJewel(aOpMode, rr_Constants.AllianceColorEnum.BLUE);
        Thread.sleep(300);

        switch(robot.getPictograph(aOpMode)) {
            case RIGHT: columnDistance = 5 + 7; moveDirection = rr_Constants.DirectionEnum.SidewaysRight; break; // 7 inches between cube columns
            case CENTER: columnDistance = 5; moveDirection = rr_Constants.DirectionEnum.SidewaysRight; break; // 7 inches between cube columns
            case LEFT: columnDistance = 2; moveDirection = rr_Constants.DirectionEnum.SidewaysLeft; break; // 7 inches between cube columns
            default: columnDistance = 5; moveDirection = rr_Constants.DirectionEnum.SidewaysRight; break;
        }

        moveWheels(aOpMode, 23, .4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(300);
        moveWheels(aOpMode, columnDistance, .4f, moveDirection, true);
        Thread.sleep(300);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 20);
        Thread.sleep(300);
        robot.moveCubeArmToPositionWithLimits(aOpMode, CUBE_ARM_GRAB, CUBE_ARM_LOWER_POWER);
        Thread.sleep(300);
        universalMoveRobot(aOpMode, 20, .3f, 0, 1500, falseStop, false, 0, 0);
        Thread.sleep(300);

    }

    // robot starts closer to audience
    public void redOneAutonomousCommonAction(rr_OpMode aOpMode) throws InterruptedException {

        float columnDistance;

        robot.closeCubeClawServoOneCube();
        robot.moveCubeArmToPositionWithTouchLimits(aOpMode, rr_Constants.CUBE_ARM_MIDDLE, CUBE_ARM_RAISE_POWER);


        aOpMode.DBG("In Red One Common");
        detectColorAndPushJewel(aOpMode, rr_Constants.AllianceColorEnum.RED);
        Thread.sleep(300);

        switch(robot.getPictograph(aOpMode)) {
            case RIGHT: columnDistance = 32 - 7; break; // 7 inches between cube columns
            case CENTER: columnDistance = 32; break; // 7 inches between cube columns
            case LEFT: columnDistance = 32 + 7; break; // 7 inches between cube columns
            default: columnDistance = 32; break;
        }

        moveWheels(aOpMode, columnDistance, .4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(300);
        moveWheels(aOpMode, 6, .4f, rr_Constants.DirectionEnum.SidewaysRight, true);
        Thread.sleep(300);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, -90);
        Thread.sleep(300);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, -90);
        Thread.sleep(300);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, -110);
        Thread.sleep(300);
        robot.moveCubeArmToPositionWithLimits(aOpMode, CUBE_ARM_GRAB, CUBE_ARM_LOWER_POWER);
        Thread.sleep(300);
        universalMoveRobot(aOpMode, 20, .3f, 0, 1500, falseStop , false, 0, 0);
        Thread.sleep(300);


        // universalMoveRobot - backwards until red line is detected
        // Move forward / backward based on Vuforia pattern detected
        // rotate counterclockwise 90 degrees
    }

    // robot starts farther away from audience
    public void redTwoAutonomousCommonAction(rr_OpMode aOpMode)throws InterruptedException{

        float columnDistance;
        rr_Constants.DirectionEnum moveDirection;

        robot.closeCubeClawServoOneCube();
        robot.moveCubeArmToPositionWithTouchLimits(aOpMode, rr_Constants.CUBE_ARM_MIDDLE, CUBE_ARM_RAISE_POWER);

        detectColorAndPushJewel(aOpMode, rr_Constants.AllianceColorEnum.RED);

        switch(robot.getPictograph(aOpMode)) {
            case RIGHT: columnDistance = 10; moveDirection = rr_Constants.DirectionEnum.SidewaysRight; break; // 7 inches between cube columns
            case CENTER: columnDistance = 10 - 7; moveDirection = rr_Constants.DirectionEnum.SidewaysRight; break; // 7 inches between cube columns
            case LEFT: columnDistance = 4; moveDirection = rr_Constants.DirectionEnum.SidewaysLeft; break; // 7 inches between cube columns
            default: columnDistance = 10; moveDirection = rr_Constants.DirectionEnum.SidewaysRight; break;
        }

        Thread.sleep(300);
        moveWheels(aOpMode, 26, .3f, rr_Constants.DirectionEnum.SidewaysRight, true);
        Thread.sleep(300);
        moveWheels(aOpMode, 15, .4f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(300);
        robot.turnUsingEncoders(aOpMode, 180, .3f, rr_Constants.TurnDirectionEnum.Clockwise);
        Thread.sleep(300);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 180);
        Thread.sleep(300);
        moveWheels(aOpMode, 12, .4f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(300);
        moveWheels(aOpMode, columnDistance, .4f, moveDirection, true);
        Thread.sleep(300);
        robot.turnAbsoluteBoschGyroDegrees(aOpMode, 200);
        Thread.sleep(300);
        robot.moveCubeArmToPositionWithLimits(aOpMode, CUBE_ARM_GRAB, CUBE_ARM_LOWER_POWER);
        Thread.sleep(300);
        universalMoveRobot(aOpMode, 20, .3f, 0, 1500, falseStop , false, 0, 0);
        Thread.sleep(300);

    }

    public void detectColorAndPushJewel(rr_OpMode aOpMode, rr_Constants.AllianceColorEnum teamColor) throws InterruptedException {

        robot.setJewelArmDownRead();

        Thread.sleep(500);

        rr_Constants.JewelColorEnum leftJewelColor = robot.getJewelLeftColor(aOpMode);
        rr_Constants.JewelColorEnum rightJewelColor = robot.getJewelRightColor(aOpMode);

        aOpMode.DBG("In detectColorAndPushJewel");


        if (teamColor == rr_Constants.AllianceColorEnum.BLUE) {
            aOpMode.DBG("In detectColorAndPushJewel Blue Alliance");

            if ((leftJewelColor == rr_Constants.JewelColorEnum.BLUE) &&
                    (rightJewelColor == rr_Constants.JewelColorEnum.RED)) {
                aOpMode.telemetryAddData("LEFT IS BLUE","LEFT_BLUE", "Left is Blue");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushRightJewel();
                if (leftJewelColor == rr_Constants.JewelColorEnum.RED) {
                    robot.pushLeftJewel();
                }
            } else if ((leftJewelColor == rr_Constants.JewelColorEnum.RED) &&
                    (rightJewelColor == rr_Constants.JewelColorEnum.BLUE)) {
                 aOpMode.telemetryAddData("RIGHT IS BLUE","RIGHT_BLUE", "RIGHT is Blue");
                 aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushLeftJewel();
                if (rightJewelColor == rr_Constants.JewelColorEnum.RED) {
                    robot.pushRightJewel();
                }
            }
            else if(((leftJewelColor) == rr_Constants.JewelColorEnum.UNKNOWN) ||
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
                 aOpMode.telemetryAddData("RIGHT IS RED","RIGHT_RED", "Right is RED");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushLeftJewel();
                if (rightJewelColor == rr_Constants.JewelColorEnum.BLUE) {
                    robot.pushRightJewel();
                }
            } else if ((leftJewelColor == rr_Constants.JewelColorEnum.RED) &&
                    (rightJewelColor== rr_Constants.JewelColorEnum.BLUE)) {
                 aOpMode.telemetryAddData("LEFT IS RED","LEFT_RED", "Left is RED");
                aOpMode.telemetryUpdate();
                robot.setJewelArmDownPush();
                robot.pushRightJewel();
                if (leftJewelColor == rr_Constants.JewelColorEnum.BLUE) {
                    robot.pushLeftJewel();
                }
            }
            else if((leftJewelColor == rr_Constants.JewelColorEnum.UNKNOWN) ||
                    ((rightJewelColor) == rr_Constants.JewelColorEnum.UNKNOWN)) {
                aOpMode.telemetryAddData("Unknown: RedTeam", "Unknown", "No color detected");
                aOpMode.telemetryAddData("Left color", "left color", "Left color" + robot.getJewelLeftColor(aOpMode));
                aOpMode.telemetryAddData("Right color", "Right color", "Right color" + robot.getJewelRightColor(aOpMode));
                aOpMode.telemetryUpdate();
            }
        }

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
            moveRobotToPositionFB(aOpMode, distance, power, isRampedPower);
        } else if (Direction == Backward) {
            moveRobotToPositionFB(aOpMode, -distance, power, isRampedPower);
        } else if (Direction == rr_Constants.DirectionEnum.SidewaysLeft) {
            moveRobotToPositionSideways(aOpMode, distance, power, isRampedPower);
        } else if (Direction == rr_Constants.DirectionEnum.SidewaysRight) {
            moveRobotToPositionSideways(aOpMode, -distance, power, isRampedPower);
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
        targetPosition = (int) ((distance / (Math.PI * MECANUM_WHEEL_DIAMETER)) * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION);
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


    public class blueLineDetectCondition implements rr_OpMode.StopCondition {

        public boolean stopCondition(rr_OpMode aOpMode) throws InterruptedException {
            return ((robot.getFloorBlueReading() >= (rr_Constants.FLOOR_BLUE_THRESHOLD)));
        }
    }

    public class redLineDetectCondition implements rr_OpMode.StopCondition {

        public boolean stopCondition(rr_OpMode aOpMode) throws InterruptedException {
            return ((robot.getFloorRedReading() >= (rr_Constants.FLOOR_RED_THRESHOLD)));
        }
    }


    public class FalseCondition implements rr_OpMode.StopCondition {
        //can be used as an empty condition, so the robot keeps running in universal movement
        public boolean stopCondition(rr_OpMode aOpMode) throws InterruptedException {
            return (false);
        }
    }
//
//
//    public class EopdProximityCondition implements vv_OpMode.StopCondition {
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            return (getEopdRawValue(aOpMode) > EOPD_PROXIMITY_THRESHOLD);
//        }
//    }
//
//    public class RangeSensorProximityOrColorVerifiedCondition implements vv_OpMode.StopCondition {
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            return (((getOpticalDistance(aOpMode) < RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD)
//                    && getOpticalDistance(aOpMode) > 0) ||
//                    (getUltrasonicDistance(aOpMode)
//                            < RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD) ||
//                    (getBeaconLeftColor(aOpMode) == getBeaconRightColor(aOpMode)));
//
//        }
//    }
//
//    public class RangeSensorOpticalProximityCondition implements vv_OpMode.StopCondition {
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            return (((getOpticalDistance(aOpMode) < RANGESENSOR_OPTICAL_PROXIMITY_THRESHOLD)
//                    && getOpticalDistance(aOpMode) > 0));
//
//        }
//    }
//
//    public class RangeSensorUltraSonicProximityCondition implements vv_OpMode.StopCondition {
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            return (getUltrasonicDistance(aOpMode)
//                    < RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD);
//
//        }
//    }
//
//    public class RangeSensorUltraSonicCornerPositioningCondition implements vv_OpMode.StopCondition {
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//
//            //TODO: This code is not functioning because we are not facing the right wall to do this.
//            return (getUltrasonicDistance(aOpMode)
//                    < 2 * RANGESENSOR_ULTRASONIC_PROXIMITY_THRESHOLD);
//        }
//    }
//    }






}

