package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.rr_Constants.MECANUM_WHEEL_DIAMETER;


public class rr_AutoLib {

    //TODO: Declarations of Conditions

//    protected LineDetectCondition lineDetectStop = new LineDetectCondition();
//    protected FalseCondition falseStop = new FalseCondition();
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
    public void blueOneAutonomousCommonAction(rr_OpMode aOpMode) {

        // TODO: create stop condition for line
        //universalMoveRobot(aOpMode, 0, .25, 0, 5000, condition, false, 0, 0);
        // Move forward / backward based on Vuforia pattern detected

    }

    // robot starts farther away from audience
    public void blueTwoAutonomousCommonAction(rr_OpMode aOpMode) {

        // moveWheels - forward
        // universalMoveRobot - sideways right until blue line is detected
        // Move sideways left / right based on Vuforia pattern detected
    }

    // robot starts closer to audience
    public void redOneAutonomousCommonAction(rr_OpMode aOpMode){

        // universalMoveRobot - backwards until red line is detected
        // Move forward / backward based on Vuforia pattern detected
        // rotate counterclockwise 90 degrees
    }

    // robot starts farther away from audience
    public void redTwoAutonomousCommonAction(rr_OpMode aOpMode){

        // moveWheels - backward
        // universalMoveRobot - sideways right until blue line is detected
        // Move sideways left / right based on Vuforia pattern detected
        // rotate 180 degrees
    }

    public void detectColorAndPushJewel(rr_OpMode aOpMode, rr_Constants.AllianceColorEnum teamColor) throws InterruptedException {

        if (teamColor == rr_Constants.AllianceColorEnum.BLUE) {
            if ((robot.getJewelLeftColor(aOpMode) == rr_Constants.JewelColorEnum.BLUE) ||
                    (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.RED)) {
                robot.pushRightJewel();
                if (robot.getJewelLeftColor(aOpMode) == rr_Constants.JewelColorEnum.RED) {
                    robot.pushLeftJewel();
                }
            } else if ((robot.getJewelLeftColor(aOpMode) == rr_Constants.JewelColorEnum.RED) ||
                    (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.BLUE)) {
                robot.pushLeftJewel();
                if (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.RED) {
                    robot.pushRightJewel();
                }
            }
        }

        if (teamColor == rr_Constants.AllianceColorEnum.RED) {
            if ((robot.getJewelLeftColor(aOpMode) == rr_Constants.JewelColorEnum.BLUE) ||
                    (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.RED)) {
                robot.pushLeftJewel();
                if (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.BLUE) {
                    robot.pushRightJewel();
                }
            } else if ((robot.getJewelLeftColor(aOpMode) == rr_Constants.JewelColorEnum.RED) ||
                    (robot.getJewelRightColor(aOpMode) == rr_Constants.JewelColorEnum.BLUE)) {
                robot.pushRightJewel();
                if (robot.getJewelLeftColor(aOpMode) == rr_Constants.JewelColorEnum.BLUE) {
                    robot.pushLeftJewel();
                }
            }
        }
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


    //TODO: Conditions, we might need these. But we need to incorporate them into

//    public class LineDetectCondition implements vv_OpMode.StopCondition {
//
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            return ((getFloorColorIntensity(aOpMode) >= (floorWhiteThreshold - FLOOR_WHITE_MARGIN)));
//        }
//    }
//
//    public class FalseCondition implements vv_OpMode.StopCondition {
//        //can be used as an empty condition, so the robot keeps running in universal movement
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            return (false);
//        }
//    }
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

