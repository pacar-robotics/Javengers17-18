package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.rr_Constants.*;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.SidewaysRight;


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

    public void redAllianceJewelPush(rr_OpMode aOpMode) throws InterruptedException {
        if (robot.getJewelLeftColor(aOpMode) == JewelColorEnum.BLUE && robot.getJewelRightColor(aOpMode) == JewelColorEnum.RED) {
            robot.setJewelKnockerLeft();
        }
        else if(robot.getJewelLeftColor(aOpMode) == JewelColorEnum.RED && robot.getJewelRightColor(aOpMode) == JewelColorEnum.BLUE) {
            robot.setJewelKnockerRight();
        }
        else {
            aOpMode.telemetryAddData("Jewel", "Color Detection", "Unknown");
            aOpMode.telemetryUpdate();
        }
    }

    public void blueAllianceJewelPush(rr_OpMode aOpMode) throws InterruptedException {
        if (robot.getJewelLeftColor(aOpMode) == JewelColorEnum.BLUE && robot.getJewelRightColor(aOpMode) == JewelColorEnum.RED) {
            robot.setJewelKnockerRight();
        }
        else if(robot.getJewelLeftColor(aOpMode) == JewelColorEnum.RED && robot.getJewelRightColor(aOpMode) == JewelColorEnum.BLUE) {
            robot.setJewelKnockerLeft();
        }
        else {
            aOpMode.telemetryAddData("Jewel", "Color Detection", "Unknown");
            aOpMode.telemetryUpdate();
        }
    }

    //TODO: Jewel is similar to process below


//    public void detectColorAndPressBeacon(vv_OpMode aOpMode,
//                                          vv_Constants.BeaconColorEnum teamColor) throws
//            InterruptedException {
//
//
//        if (teamColor == vv_Constants.BeaconColorEnum.BLUE) {
//            //team blue
//            if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
//                //found blue
//                //press left beacon button
//                extendLeftBeaconButtonPress(aOpMode);
//            } else if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
//                //found red
//                //press right button
//                extendRightBeaconButtonPress(aOpMode);
//            }
//        }
//        if (teamColor == vv_Constants.BeaconColorEnum.RED) {
//            //team red
//            if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.RED) {
//                //found red
//                //press left beacon button
//                extendLeftBeaconButtonPress(aOpMode);
//            } else if (getBeaconLeftColor(aOpMode) == vv_Constants.BeaconColorEnum.BLUE) {
//                //found blue
//                //press right button
//                extendRightBeaconButtonPress(aOpMode);
//            }
////        }
//
//
//        //move forward to press beacon button.
//        //lets keep pulsing forward until the color changes or time runs out or proximity limits
//        //are reached
//
//        universalMoveRobot(aOpMode, 90, 0.3, 0.0, 2000, new
//                RangeSensorProximityOrColorVerifiedCondition(), true, 200, 10);
//
//
//        //now retract both beacon presses
//        closeLeftBeaconButtonPress(aOpMode);
//        closeRightBeaconButton(aOpMode);
//    }


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
//
//
//    public class colorPressVerifiedCondition implements vv_OpMode.StopCondition {
//        public boolean stopCondition(vv_OpMode aOpMode) throws InterruptedException {
//            //button is pressed because both colors match , not a strong test but a good starting point for
//            //teleop.
//            return (getBeaconLeftColor(aOpMode) == getBeaconRightColor(aOpMode));
//
//        }
//    }


}

