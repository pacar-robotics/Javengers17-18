package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_PUSHED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.rr_Constants.FIELD_ORIENTED_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_HIGH;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_EXTEND_UP;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_OPEN_PULSE;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.STANDARD_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_HORIZONTAL_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_1CUBE_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Robot.cubePusherStateEnum.PUSHED;
import static org.firstinspires.ftc.teamcode.rr_Robot.cubePusherStateEnum.REST;
import static org.firstinspires.ftc.teamcode.rr_Robot.intakeStateEnum.RUNNING;
import static org.firstinspires.ftc.teamcode.rr_Robot.intakeStateEnum.STOPPED;

public class rr_TeleLib {

    rr_Robot robot;
    rr_OpMode aOpMode;

    public float gamepad2PowerFactor = STANDARD_DRIVE_POWER_FACTOR;

    private boolean isGyroCalibrated = false;
    private boolean isIntake = true;
    private float turnVelocity = 0.0f;
    private double polarMagnitude = 0.0f;
    private double cubeArmPower = 0.0f;

    public rr_TeleLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode);
        this.aOpMode = aOpMode;
        robot.teleopInit(aOpMode, aHwMap);
    }


    //************ PROCESS METHODS ************//

    public void processTeleOpDrive() throws InterruptedException {
        if (!isGyroCalibrated) {
            processStandardDrive();
        } else {
            processFieldOrientedDrive();
        }
    }

    public void processFieldOrientedDrive() throws InterruptedException {
        //process joysticks

        if (Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.


            robot.universalMoveRobot(aOpMode, getXVelocity(getGamePad2LeftJoystickPolarMagnitude(aOpMode) * gamepad2PowerFactor,
                    getGamePad2LeftJoystickPolarAngle(aOpMode) - robot.getBoschGyroSensorHeading(aOpMode)),
                    getYVelocity(getGamePad2LeftJoystickPolarMagnitude(aOpMode) * gamepad2PowerFactor,
                            getGamePad2LeftJoystickPolarAngle(aOpMode) - robot.getBoschGyroSensorHeading(aOpMode)));

        } else if (Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways. We need to turn.
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * gamepad2PowerFactor;

            if (aOpMode.gamepad2.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * STANDARD_DRIVE_POWER_FACTOR;
                robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * STANDARD_DRIVE_POWER_FACTOR;
                robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else if (Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            //add to  previous turnvelocity and divide by 2, to smoothen out the turn response.
            //moves will start out slow then build.
            //moves will stop rapidly as the joystick when released will be be read as less than ANALOG_STICK_THRESHOLD
            //and motors will stop.


            //adjust the power to address the effect of checking for deadzone.
            //if the clip and scale operation is not performed, the starting
            polarMagnitude =
                    clipAndScaleTranslationPower(polarMagnitude + (getGamePad1LeftJoystickPolarMagnitude(aOpMode) * FIELD_ORIENTED_DRIVE_POWER_FACTOR)) / 2;


            robot.universalMoveRobot(aOpMode,
                    getXVelocity(polarMagnitude,
                            getGamePad1LeftJoystickPolarAngle(aOpMode) - robot.getBoschGyroSensorHeading(aOpMode)),
                    getYVelocity(polarMagnitude,
                            getGamePad1LeftJoystickPolarAngle(aOpMode) - robot.getBoschGyroSensorHeading(aOpMode)));

        } else if (Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways. We need to turn.
            //add to  previous turnvelocity and divide by 2, to smoothen out the turn response.
            //turns will start out slow then build.
            //turns will stop rapidly as the joystick when released will be be read as less than ANALOG_STICK_THRESHOLD
            //and motors will stop.

            turnVelocity = (turnVelocity +
                    (float) getGamePad1RightJoystickPolarMagnitude(aOpMode) * FIELD_ORIENTED_DRIVE_POWER_FACTOR) / 2;


            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runRampedMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runRampedMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else {
            //both joysticks on both gamepads are at rest, stop the robot.
            robot.stopBaseMotors(aOpMode);
        }

        if (aOpMode.gamepad1.dpad_up || aOpMode.gamepad2.dpad_up) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, 0);
        }
        if (aOpMode.gamepad1.dpad_right || aOpMode.gamepad2.dpad_right) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, 90);
        }
        if (aOpMode.gamepad1.dpad_down || aOpMode.gamepad2.dpad_down) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, 180);
        }
        if (aOpMode.gamepad1.dpad_left || aOpMode.gamepad2.dpad_left) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, -90);
        }
    }

    public void processIMUGyroReset() throws InterruptedException {
        if ((aOpMode.gamepad1.right_stick_button && aOpMode.gamepad1.left_stick_button) ||
                (aOpMode.gamepad2.right_stick_button && aOpMode.gamepad2.left_stick_button)) {
            robot.setBoschGyroZeroYaw(aOpMode);
            isGyroCalibrated = true;
        }
    }

    public void processStandardDrive() throws InterruptedException {
        if (Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * STANDARD_DRIVE_POWER_FACTOR;

            if (aOpMode.gamepad2.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runRampedMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runRampedMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else if (Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(aOpMode,
                    aOpMode.gamepad2.left_stick_x * STANDARD_DRIVE_POWER_FACTOR,
                    -aOpMode.gamepad2.left_stick_y * STANDARD_DRIVE_POWER_FACTOR);

        } else if (Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad1RightJoystickPolarMagnitude(aOpMode) * TURN_POWER_FACTOR;

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runRampedMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runRampedMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else if (Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(aOpMode,
                    aOpMode.gamepad1.left_stick_x * STANDARD_DRIVE_POWER_FACTOR,
                    -aOpMode.gamepad1.left_stick_y * STANDARD_DRIVE_POWER_FACTOR);

        } else {
            //both joysticks on both gamepads are at rest, stop the robot.
            robot.stopBaseMotors(aOpMode);
        }
    }

    public void processRelicSlide() {
        if (Math.abs(aOpMode.gamepad2.right_trigger) > TRIGGER_THRESHOLD) {
            robot.setRelicWinchPower(RELIC_WINCH_EXTEND_POWER_FACTOR);
        } else if (Math.abs(aOpMode.gamepad2.left_trigger) > TRIGGER_THRESHOLD) {
            robot.setRelicWinchPower(RELIC_WINCH_RETRACT_POWER_FACTOR);
        } else {
            //the triggers are in dead zone.
            //stop the relic slide
            robot.setRelicWinchPower(0);
        }
    }

    public void processRelicClaw() throws InterruptedException {
        if (aOpMode.gamepad2.x) {
            robot.setRelicClawClosed();
        }
        if (aOpMode.gamepad2.b) {
            robot.setRelicClawOpen();
        }
        if (aOpMode.gamepad2.y) {
            robot.setRelicClawPosition(RELIC_ARM_OPEN_PULSE);
            Thread.sleep(250);
            robot.setRelicClawPosition(RELIC_CLAW_CLOSED);
        }
    }

    public void processRelicHand() throws InterruptedException {
        if (aOpMode.gamepad2.a) {
            robot.setRelicArmPosition(RELIC_ARM_GRAB);
        }
        if (aOpMode.gamepad2.right_bumper) {
            robot.setRelicArmPosition(RELIC_ARM_EXTEND_UP);
        }

    }

    public void processBalance() throws InterruptedException {
        if (aOpMode.gamepad2.left_bumper) {
            robot.moveRobotToPositionFB(aOpMode, -20.5f, 1.0f, true);
            gamepad2PowerFactor = 0.35f;
        }
    }

    public void processIntake() throws InterruptedException {
        //checks if the alternate button is pressed
        if (aOpMode.gamepad1.left_bumper && aOpMode.gamepad1.right_bumper) {
            if (robot.intakeState == STOPPED) {
                robot.intakeState = RUNNING;
                isIntake = false;

                Thread.sleep(500); //absorb the extra key presses

            } else {
                robot.setIntakePower(this.aOpMode, 0, 0);
                robot.intakeState = STOPPED;
                Thread.sleep(500); //absorb the extra key presses
            }
        } else if (aOpMode.gamepad1.right_bumper) {
            if (robot.intakeState == STOPPED) {
                robot.intakeState = RUNNING;
                isIntake = true;

                Thread.sleep(500); //absorb the extra key presses

            } else {
                robot.setIntakePower(this.aOpMode, 0, 0);
                robot.intakeState = STOPPED;
                Thread.sleep(500); //absorb the extra key presses
            }
        }
        if(robot.intakeState==RUNNING){
            runIntakeWithDiagonalCheck(aOpMode, isIntake); //check for cubes going in sideways
            //so we can counter rotate to straighted
        }
        if(aOpMode.gamepad1.left_bumper){
            if(robot.cubePusherState== REST){
                robot.cubePusherState=PUSHED;
                robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_PUSHED_POSITION);
            }else{
                robot.cubePusherState=REST;
                robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_RESTED_POSITION);
            }
        }
    }

    /*
    public void processTrayLift() throws InterruptedException {
        if (aOpMode.gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
            robot.setTrayLiftPower(aOpMode, aOpMode.gamepad1.left_trigger * TRAY_LIFT_POWER_FACTOR);
        } else if (aOpMode.gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            robot.setTrayLiftPower(aOpMode, aOpMode.gamepad1.right_trigger * TRAY_LIFT_POWER_FACTOR);
        } else if (aOpMode.gamepad1.x){
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_COLLECTION_POSITION, TRAY_LIFT_POWER);
        } else if (aOpMode.gamepad1.b){
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_1CUBE_POSITION, TRAY_LIFT_POWER);
        } else if (aOpMode.gamepad1.y){
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_MAX_POSITION, TRAY_LIFT_POWER);
        } else {
            robot.setTrayLiftPower(aOpMode, 0);
        }
    }

    */
    public void processTrayLift(rr_OpMode aOpMode) throws InterruptedException {
        robot.trayHeightPosition=robot.getTrayPosition(aOpMode);
        if(aOpMode.gamepad1.right_trigger>TRIGGER_THRESHOLD){
            //raise the height of the tray
            if(robot.trayHeightPosition>=TRAY_HEIGHT_MAX_POSITION-50){ //check for limit
               robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_MAX_POSITION, TRAY_LIFT_POWER);
            }else{
                robot.setTrayHeightPositionWithTouchLimits(aOpMode, robot.trayHeightPosition+50, TRAY_LIFT_POWER);
            }
        }
        if(aOpMode.gamepad1.left_trigger>TRIGGER_THRESHOLD){
            //lower the height of the tray
            if(robot.trayHeightPosition<=TRAY_HEIGHT_COLLECTION_POSITION+50){ //check for limit
                robot.setTrayHeightPositionWithTouchLimits(aOpMode,TRAY_HEIGHT_COLLECTION_POSITION, TRAY_LIFT_POWER);
            }else{
                robot.setTrayHeightPositionWithTouchLimits(aOpMode, robot.trayHeightPosition-50, TRAY_LIFT_POWER);
            }

        } else if (aOpMode.gamepad1.x){
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_COLLECTION_POSITION, TRAY_LIFT_POWER);
            robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_COLLECTION_POSITION);

        } else if (aOpMode.gamepad1.b){
            //flip tray horizontal and go to scoring depending on position
            if(robot.trayFlipPosition==TRAY_FLIP_COLLECTION_POSITION){
                robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_HORIZONTAL_POSITION);
            }else{
                robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_SCORING_POSITION);
            }
        } else if (aOpMode.gamepad1.y){
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_MAX_POSITION, TRAY_LIFT_POWER);
        } else if (aOpMode.gamepad1.a){
        robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_1CUBE_POSITION, TRAY_LIFT_POWER);
    }
}


    public void printTelemetry() throws InterruptedException {
        if (DEBUG) {
            aOpMode.telemetry.addLine("Tray Arm Pos: " + robot.getMotorPosition(aOpMode, TRAY_LIFT_MOTOR));
            aOpMode.telemetryAddLine("BRPower" + robot.getMotorPower(aOpMode, BACK_RIGHT_MOTOR));
            aOpMode.telemetryAddLine("FRPower" + robot.getMotorPower(aOpMode, FRONT_LEFT_MOTOR));
            aOpMode.telemetryAddLine("BLPower" + robot.getMotorPower(aOpMode, BACK_LEFT_MOTOR));
            aOpMode.telemetryAddLine("FLPower" + robot.getMotorPower(aOpMode, FRONT_LEFT_MOTOR));
            aOpMode.telemetryUpdate();
        }
    }


    //************ JOYSTICK INPUT CONVERSION ************//

    public double getGamePad1RightJoystickPolarMagnitude(rr_OpMode aOpMode) {
        //returns the magnitude of the polar vector for the rotation calculations
        //for field oriented drive
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.right_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.sqrt(Math.pow(aOpMode.gamepad1.right_stick_x, 2.0) +
                    Math.pow(-aOpMode.gamepad1.right_stick_y, 2.0)));
        } else {
            return 0;
        }

    }

    public double getGamePad1RightJoystickPolarAngle(rr_OpMode aOpMode) {
        //returns polar angle in degrees of vector for the rotation calculations
        //for field oriented drive.
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.right_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.toDegrees(Math.atan2(aOpMode.gamepad1.right_stick_x,
                    -aOpMode.gamepad1.right_stick_y)));
        } else {
            return 0;
        }
    }

    public double getGamePad1LeftJoystickPolarMagnitude(rr_OpMode aOpMode) {
        //returns the magnitude of the polar vector for the rotation calculations
        //for field oriented drive
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.sqrt(Math.pow(aOpMode.gamepad1.left_stick_x, 2.0) +
                    Math.pow(-aOpMode.gamepad1.left_stick_y, 2.0)));
        } else {
            return 0;
        }
    }

    public double getGamePad1LeftJoystickPolarAngle(rr_OpMode aOpMode) {
        //returns polar angle in degrees of vector for the rotation calculations
        //for field oriented drive.
        //inverted y
        if ((Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.toDegrees(Math.atan2(aOpMode.gamepad1.left_stick_x,
                    -aOpMode.gamepad1.left_stick_y)));
        } else {
            return 0;
        }
    }

    public double getGamePad2RightJoystickPolarMagnitude(rr_OpMode aOpMode) {
        //returns the magnitude of the polar vector for the rotation calculations
        //for field oriented drive
        //inverted y
        if ((Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad2.right_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.sqrt(Math.pow(aOpMode.gamepad2.right_stick_x, 2.0) +
                    Math.pow(-aOpMode.gamepad2.right_stick_y, 2.0)));
        } else {
            return 0;
        }

    }

    public double getGamePad2RightJoystickPolarAngle(rr_OpMode aOpMode) {
        //returns polar angle in degrees of vector for the rotation calculations
        //for field oriented drive.
        //inverted y
        if ((Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad2.right_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.toDegrees(Math.atan2(aOpMode.gamepad2.right_stick_x,
                    -aOpMode.gamepad2.right_stick_y)));
        } else {
            return 0;
        }
    }

    public double getGamePad2LeftJoystickPolarMagnitude(rr_OpMode aOpMode) {
        //returns the magnitude of the polar vector for the rotation calculations
        //for field oriented drive
        //inverted y
        if ((Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.sqrt(Math.pow(aOpMode.gamepad2.left_stick_x, 2.0) +
                    Math.pow(-aOpMode.gamepad2.left_stick_y, 2.0)));
        } else {
            return 0;
        }
    }

    public double getGamePad2LeftJoystickPolarAngle(rr_OpMode aOpMode) {
        //returns polar angle in degrees of vector for the rotation calculations
        //for field oriented drive.
        //inverted y
        if ((Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD) ||
                (Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD)) {
            return (Math.toDegrees(Math.atan2(aOpMode.gamepad2.left_stick_x,
                    -aOpMode.gamepad2.left_stick_y)));
        } else {
            return 0;
        }
    }

    public double getXVelocity(double polarMagnitude, double polarAngle) {
        return polarMagnitude * Math.sin(Math.toRadians(polarAngle));
    }

    public double getYVelocity(double polarMagnitude, double polarAngle) {
        return polarMagnitude * Math.cos(Math.toRadians(polarAngle));
    }

    public double clipAndScaleTranslationPower(double magnitude) {
        magnitude = magnitude - ANALOG_STICK_THRESHOLD + MOTOR_LOWER_POWER_THRESHOLD;
        if (magnitude > 1.0f) {
            magnitude = 1.0f;
        }
        return magnitude;
    }

    public double clipAndScaleTrayLiftPower(double magnitude) {
        magnitude = magnitude - TRIGGER_THRESHOLD + MOTOR_LOWER_POWER_THRESHOLD;
        if (magnitude > 1.0f) {
            magnitude = 1.0f;
        } else if (magnitude < MOTOR_LOWER_POWER_THRESHOLD) {
            magnitude = TRAY_LIFT_POWER;
        }

        aOpMode.telemetryAddData("Tray Lift Power", "trayLiftPower", "[" + magnitude + "]");
        return magnitude;
    }

    public void runIntakeWithDiagonalCheck(rr_OpMode aOpMode, boolean isOuttake) throws InterruptedException {
        int polarity = 0;

        if(isOuttake) {
            polarity = 1;
        } else {
            polarity = -1;
        }
        if ((robot.getIntakeUltrasonicRightSensorRange(aOpMode) < 5)
                || robot.getIntakeOpticalRightSensorRange(aOpMode) < 2)

        {
            //the cube is going in sideways
            //we should switch to low power but reverse one of the motors
            //this should cause the motors to rotate the cube so it is straight
            //this has to be tested and adjusted.
            robot.setIntakePower(aOpMode, INTAKE_POWER_HIGH, -INTAKE_POWER_HIGH);
            Thread.sleep(200); //wait for a little time for rotation.
            robot.setIntakePower(aOpMode, 0, 0);
            Thread.sleep(100); //wait for a second for rotation.
        } else

        {
            robot.setIntakePower(aOpMode, polarity * INTAKE_POWER_HIGH,
                    polarity * INTAKE_POWER_HIGH);
        }
    }

    private void processTrayHeightAdjustments(rr_OpMode aOpMode) throws InterruptedException{

        if(aOpMode.gamepad1.right_trigger>TRIGGER_THRESHOLD){
            //raise the height of the tray
            if(robot.trayHeightPosition>=TRAY_HEIGHT_MAX_POSITION){ //check for limit
                robot.trayHeightPosition=TRAY_HEIGHT_MAX_POSITION;
            }else{
                robot.trayHeightPosition+=0.025;
            }
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, robot.trayHeightPosition, TRAY_LIFT_POWER);
        }
        if(aOpMode.gamepad1.left_trigger>TRIGGER_THRESHOLD){
            //lower the height of the tray
            if(robot.trayHeightPosition<=TRAY_HEIGHT_COLLECTION_POSITION){ //check for limit
                robot.trayHeightPosition=TRAY_HEIGHT_COLLECTION_POSITION;
            }else{
                robot.trayHeightPosition-=0.025;
            }
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, robot.trayHeightPosition, TRAY_LIFT_POWER);
        }





    }

    private void processTrayFlipAdjustments(rr_OpMode aOpMode) throws InterruptedException{
        if(aOpMode.gamepad1.y){
            //increase angle of tray flip to be more vertical.
            if(robot.trayFlipPosition<=TRAY_FLIP_SCORING_POSITION){ //check for limit
                robot.trayFlipPosition=TRAY_FLIP_SCORING_POSITION;
            }else{
                robot.trayFlipPosition-=0.025;
            }
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.a){
            //decrease angle of tray flip to be more vertical.
            if(robot.trayFlipPosition>=TRAY_FLIP_COLLECTION_POSITION){ //check for limit
                robot.trayFlipPosition=TRAY_FLIP_COLLECTION_POSITION;
            }else{
                robot.trayFlipPosition+=0.025;
            }
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.x){
            //set tray to collection
            robot.trayFlipPosition=TRAY_FLIP_COLLECTION_POSITION;
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.b){
            //set tray to scoring
            robot.trayFlipPosition=TRAY_FLIP_SCORING_POSITION;
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.left_bumper){
            //set tray to horizontal
            robot.trayFlipPosition=TRAY_FLIP_HORIZONTAL_POSITION;
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }

    }

}
