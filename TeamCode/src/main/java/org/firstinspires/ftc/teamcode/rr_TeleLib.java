package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_HOLDER_HOLD_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_HOLDER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_HOLDER_RELEASE_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_PUSHED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.rr_Constants.FIELD_ORIENTED_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_HIGH;
import static org.firstinspires.ftc.teamcode.rr_Constants.MOTOR_LOWER_POWER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_CARRY;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_DROPOFF_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_PICKUP;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WRIST_CARRY;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WRIST_DROPOFF_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WRIST_PICKUP;
import static org.firstinspires.ftc.teamcode.rr_Constants.SCORING_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.STANDARD_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.SUPPRESS_COUNTER_ROTATION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_CRYPTO_ALIGN_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_HORIZONTAL_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_2CUBE_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;


public class rr_TeleLib {

    rr_Robot robot;
    rr_OpMode aOpMode;

    public float gamepad2PowerFactor = STANDARD_DRIVE_POWER_FACTOR;

    private boolean isGyroCalibrated = false;
    private boolean isIntake = true;
    private rr_Constants.IntakeStateEnum intakeState= rr_Constants.IntakeStateEnum.STOPPED;
    private float turnVelocity = 0.0f;
    private double polarMagnitude = 0.0f;
    private double heading = 0.0f;
    private double polarAngle = 0.0f;
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

        boolean inScoringMode=
                (robot.trayFlipPosition==TRAY_FLIP_CRYPTO_ALIGN_POSITION)||
                        (robot.trayHeightPosition!=TRAY_HEIGHT_COLLECTION_POSITION); //sets the mode

        float gamePad1PowerFactor=inScoringMode ?SCORING_DRIVE_POWER_FACTOR:FIELD_ORIENTED_DRIVE_POWER_FACTOR;

        if (Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {

            //this is for the second game pad during the relic drive.

            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.


            robot.universalMoveRobot(aOpMode, getXVelocity(getGamePad2LeftJoystickPolarMagnitude(aOpMode) * gamepad2PowerFactor,
                    getGamePad2LeftJoystickPolarAngle(aOpMode) - robot.getBoschGyroSensorHeading(aOpMode)),
                    getYVelocity(getGamePad2LeftJoystickPolarMagnitude(aOpMode) * RELIC_DRIVE_POWER_FACTOR,
                            getGamePad2LeftJoystickPolarAngle(aOpMode) - robot.getBoschGyroSensorHeading(aOpMode)));

        } else if (Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways. We need to turn.
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * RELIC_DRIVE_POWER_FACTOR;

            if (aOpMode.gamepad2.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * RELIC_DRIVE_POWER_FACTOR;
                robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * RELIC_DRIVE_POWER_FACTOR;
                robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else if (Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //this is for the main drive while scoring glyphs
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            //add to  previous turnvelocity and divide by 2, to smoothen out the turn response.
            //moves will start out slow then build.
            //moves will stop rapidly as the joystick when released will be be read as less than ANALOG_STICK_THRESHOLD
            //and motors will stop.


            //adjust the power to address the effect of checking for deadzone.
            //if the clip and scale operation is not performed, the starting
            polarMagnitude =
                    (polarMagnitude+(float) getGamePad1LeftJoystickPolarMagnitude(aOpMode)
                            * gamePad1PowerFactor)/2;

            heading = robot.getBoschGyroSensorHeading(aOpMode);
            polarAngle = getGamePad1LeftJoystickPolarAngle(aOpMode) - heading;


            robot.universalMoveRobot(aOpMode,
                    getXVelocity(polarMagnitude, polarAngle),
                    getYVelocity(polarMagnitude, polarAngle));

        } else if (Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways. We need to turn.
            //add to  previous turnvelocity and divide by 2, to smoothen out the turn response.
            //turns will start out slow then build.
            //turns will stop rapidly as the joystick when released will be be read as less than ANALOG_STICK_THRESHOLD
            //and motors will stop.

            turnVelocity = (turnVelocity +
                    (float) getGamePad1RightJoystickPolarMagnitude(aOpMode) * gamePad1PowerFactor) / 2;


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


        if (aOpMode.gamepad1.dpad_up) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, 0);
        }
        if (aOpMode.gamepad1.dpad_right ) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, 90);
        }
        if (aOpMode.gamepad1.dpad_down) {
            robot.turnAbsoluteBoschGyroDegrees(aOpMode, 180);
        }
        if (aOpMode.gamepad1.dpad_left ) {
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

    }

    public void processRelicArm() throws InterruptedException {
        if (aOpMode.gamepad2.a) {
            robot.setRelicClawOpen();
            robot.setRelicArmPosition(RELIC_ARM_PICKUP);
            robot.setRelicWristPosition(RELIC_WRIST_PICKUP);

        }
        if (aOpMode.gamepad2.right_bumper) {
            //release the cube.
            robot.setRelicClawOpen();
            Thread.sleep(200);
            robot.setRelicWristPosition(robot.relicWristPosition+0.125f);
            Thread.sleep(200);
            robot.setRelicArmPosition(robot.relicArmPosition-0.1f);
            Thread.sleep(200);
        }

    }

    public void processRelicDropoff() throws InterruptedException {
        if (aOpMode.gamepad2.y) {
            //set the arm and wrist to a position just above ground, relic
            //pointing down.
            robot.setRelicWristPosition(RELIC_WRIST_DROPOFF_INIT);
            robot.setRelicArmPosition(RELIC_ARM_DROPOFF_INIT);

        }else if(aOpMode.gamepad2.dpad_up){
            robot.setRelicWristPosition(robot.relicWristPosition+0.11f);
            robot.setRelicArmPosition(robot.relicArmPosition-0.05f);

        }else if(aOpMode.gamepad2.dpad_down){

                robot.setRelicWristPosition(robot.relicWristPosition - 0.11f);
                robot.setRelicArmPosition(robot.relicArmPosition + 0.05f);


        }
    }

    public void processRelicArmAdjustments() throws InterruptedException{
        if(aOpMode.gamepad2.dpad_up){
            robot.setRelicArmPosition(robot.relicArmPosition-0.05f);
            Thread.sleep(200);
        }else if(aOpMode.gamepad2.dpad_down){
            robot.setRelicArmPosition(robot.relicArmPosition+0.05f);
            Thread.sleep(200);
        }
    }

    public void processRelicWristAdjustments() throws InterruptedException{
        if(aOpMode.gamepad2.dpad_right){
            robot.setRelicWristPosition(robot.relicWristPosition+0.05f);
            Thread.sleep(200);
        }else if(aOpMode.gamepad2.dpad_left){
            robot.setRelicWristPosition(robot.relicWristPosition-0.05f);
            Thread.sleep(200);
        }
    }


    public void processBalance() throws InterruptedException {
        if (aOpMode.gamepad2.left_bumper) {
            robot.moveRobotToPositionFB(aOpMode, -20.5f, 1.0f, true);
            gamepad2PowerFactor = 0.35f;
        }
    }

    public void processIntake() throws InterruptedException {
        if (aOpMode.gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if(intakeState== rr_Constants.IntakeStateEnum.INTAKE) {
               intakeState= rr_Constants.IntakeStateEnum.STOPPED;
            }else{
                intakeState= rr_Constants.IntakeStateEnum.INTAKE;
            }
            Thread.sleep(250); //to absorb button presses.
        }
        if(aOpMode.gamepad1.left_trigger >TRIGGER_THRESHOLD) {
            if (intakeState == rr_Constants.IntakeStateEnum.OUTTAKE) {
                intakeState = rr_Constants.IntakeStateEnum.STOPPED;
            } else {
                intakeState = rr_Constants.IntakeStateEnum.OUTTAKE;
            }
            Thread.sleep(250); //to absorb button presses.
        }
        if(intakeState== rr_Constants.IntakeStateEnum.STOPPED){
            stopIntake(aOpMode);
        }else if(intakeState== rr_Constants.IntakeStateEnum.OUTTAKE){
            robot.setIntakePower(aOpMode,
                    -1*INTAKE_POWER_HIGH, -1*INTAKE_POWER_HIGH);
        }else {
            //when running intake forward
            runIntakeWithDiagonalCheck(aOpMode, intakeState);
        }


    }

    public void processCubeAlignment() throws InterruptedException{
    //push cubes into tray for alignment
        if (aOpMode.gamepad1.right_bumper) {
            alignCubes(aOpMode);
        }
    }

    public void processTrayLift(rr_OpMode aOpMode) throws InterruptedException {
        if (aOpMode.gamepad1.x) {
            //stop the base motors
            robot.stopBaseMotors(aOpMode);
            //prepare to score, stop intake, move tray to horizontal etc.
            prepareToScore(aOpMode);
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_2CUBE_POSITION, TRAY_LIFT_POWER);
            Thread.sleep(250); // to absorb key presses

        } else if (aOpMode.gamepad1.b) {
            //stop the base motors
            robot.stopBaseMotors(aOpMode);
            //flip tray horizontal and go to scoring depending on position
            //first lets stop the intake.
            stopIntake(aOpMode);

            if (robot.trayFlipPosition == TRAY_FLIP_COLLECTION_POSITION) {
                prepareToScore(aOpMode);
            }else if (robot.trayFlipPosition == TRAY_FLIP_HORIZONTAL_POSITION) {
                //move tray to align to cryptobox position
                robot.setCubeHolderPosition(aOpMode, CUBE_HOLDER_HOLD_POSITION);
                robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_CRYPTO_ALIGN_POSITION);
                Thread.sleep(500);
            }else if (robot.trayFlipPosition==TRAY_FLIP_CRYPTO_ALIGN_POSITION){
               robot.setCubeHolderPosition(aOpMode,CUBE_HOLDER_RELEASE_POSITION);
                robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_SCORING_POSITION);
                Thread.sleep(500);
                //return to collection position
                prepareToCollect(aOpMode);
            }else if (robot.trayFlipPosition == TRAY_FLIP_SCORING_POSITION) {
                //probably not used because of auto return to collection
                prepareToCollect(aOpMode);
            }
            Thread.sleep(250); // to absorb key presses

        } else if (aOpMode.gamepad1.y) {
            //stop the base motors
            robot.stopBaseMotors(aOpMode);
            //prepare to score, stop intake, move tray to horizontal etc.
            prepareToScore(aOpMode);
            robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_MAX_POSITION, TRAY_LIFT_POWER);
            Thread.sleep(250); // to absorb key presses
        } else if (aOpMode.gamepad1.a) {
            //stop the base motors
            robot.stopBaseMotors(aOpMode);
            prepareToCollect(aOpMode);
            Thread.sleep(250); // to absorb key presses
        } else if (aOpMode.gamepad1.left_bumper) {
            //alt key mode
//stop the base motors
            robot.stopBaseMotors(aOpMode);

            if (aOpMode.gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                robot.trayHeightPosition = robot.getTrayPosition(aOpMode);
                //raise the height of the tray
                if (robot.trayHeightPosition >= TRAY_HEIGHT_MAX_POSITION - 50) { //check for limit
                    robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_MAX_POSITION, TRAY_LIFT_POWER);
                } else {
                    robot.setTrayHeightPositionWithTouchLimits(aOpMode, robot.trayHeightPosition + 50, TRAY_LIFT_POWER);
                }
                Thread.sleep(250); // to absorb key presses
            }else if (aOpMode.gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                robot.trayHeightPosition = robot.getTrayPosition(aOpMode);
                //lower the height of the tray
                if (robot.trayHeightPosition <= TRAY_HEIGHT_COLLECTION_POSITION + 50) { //check for limit
                    robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_COLLECTION_POSITION, TRAY_LIFT_POWER);
                } else {
                    robot.setTrayHeightPositionWithTouchLimits(aOpMode, robot.trayHeightPosition - 50, TRAY_LIFT_POWER);
                }
                Thread.sleep(250); // to absorb key presses

            }else if(aOpMode.gamepad1.b){

                //score slowly to let the cubes fall down.
                alignCubes(aOpMode);
                for(float f=TRAY_FLIP_COLLECTION_POSITION;f<TRAY_FLIP_SCORING_POSITION;f=-0.1f){
                    robot.setTrayFlipPosition(aOpMode,f);
                    Thread.sleep(50);
                }
                prepareToCollect(aOpMode);
                Thread.sleep(250); // to absorb key presses


            }

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
        }

        aOpMode.telemetryAddData("Tray Lift Power", "trayLiftPower", "[" + magnitude + "]");
        return magnitude;
    }

    public void runIntakeWithDiagonalCheck(rr_OpMode aOpMode, rr_Constants.IntakeStateEnum intakeState)
            throws InterruptedException {
        int polarity = 0;

        if(intakeState== rr_Constants.IntakeStateEnum.INTAKE) {
            polarity = 1;
        } else {
            polarity = -1;
        }
        double  opticalRange=robot.getIntakeOpticalRightSensorRange(aOpMode);
        double  ultrasonicRange=robot.getIntakeUltrasonicRightSensorRange(aOpMode);

        if (

                (ultrasonicRange > 14)
        ||(ultrasonicRange <4)||SUPPRESS_COUNTER_ROTATION //if set, will skip counter rotation
                )

        {
            //cube is straight pass on straight
            robot.setIntakePower(aOpMode,
                    polarity*INTAKE_POWER_HIGH, polarity*INTAKE_POWER_HIGH);

        } else

        {

            //the cube is going in sideways
            //we should switch to low power but reverse one of the motors
            //this should cause the motors to rotate the cube so it is straight
            //this has to be tested and adjusted.
            robot.setIntakePower(aOpMode, INTAKE_POWER_HIGH, -INTAKE_POWER_HIGH);
            //Thread.sleep(200); //wait for a little time for rotation.
            robot.setIntakePower(aOpMode, 0, 0);
            //Thread.sleep(100); //wait for a second for rotation.
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
            //stop the base motors.
            robot.stopBaseMotors(aOpMode);
            //increase angle of tray flip to be more vertical.
            if(robot.trayFlipPosition<=TRAY_FLIP_SCORING_POSITION){ //check for limit
                robot.trayFlipPosition=TRAY_FLIP_SCORING_POSITION;
            }else{
                robot.trayFlipPosition-=0.025;
            }
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.a){
            //stop the base motors.
            robot.stopBaseMotors(aOpMode);
            //decrease angle of tray flip to be more vertical.
            if(robot.trayFlipPosition>=TRAY_FLIP_COLLECTION_POSITION){ //check for limit
                robot.trayFlipPosition=TRAY_FLIP_COLLECTION_POSITION;
            }else{
                robot.trayFlipPosition+=0.025;
            }
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.x){
            //stop the base motors.
            robot.stopBaseMotors(aOpMode);
            //set tray to collection
            robot.trayFlipPosition=TRAY_FLIP_COLLECTION_POSITION;
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.b){
            //stop the base motors.
            robot.stopBaseMotors(aOpMode);
            //set tray to scoring
            robot.trayFlipPosition=TRAY_FLIP_SCORING_POSITION;
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }
        if(aOpMode.gamepad1.left_bumper){
            //stop the base motors.
            robot.stopBaseMotors(aOpMode);
            //set tray to horizontal
            robot.trayFlipPosition=TRAY_FLIP_HORIZONTAL_POSITION;
            robot.setTrayFlipPosition(aOpMode, robot.trayFlipPosition);
        }

    }


    public void alignCubes(rr_OpMode aOpMode) throws InterruptedException{
        robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_PUSHED_POSITION);
        Thread.sleep(500);
        robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_RESTED_POSITION);
    }

    public void stopIntake(rr_OpMode aOpMode) throws InterruptedException {
        isIntake = false;
        robot.setIntakePower(this.aOpMode, 0, 0);
    }

    public void prepareToCollect(rr_OpMode aOpMode) throws InterruptedException{
        moveWheels(aOpMode, 4, 0.5f,Forward, true);
        robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_COLLECTION_POSITION);
        robot.setTrayHeightPositionWithTouchLimits(aOpMode, TRAY_HEIGHT_COLLECTION_POSITION, TRAY_LIFT_POWER);
        robot.setCubeHolderPosition(aOpMode, CUBE_HOLDER_INIT_POSITION);
        runIntakeWithDiagonalCheck(aOpMode, rr_Constants.IntakeStateEnum.INTAKE);
    }

    public void prepareToScore(rr_OpMode aOpMode) throws InterruptedException{
        stopIntake(aOpMode);
        alignCubes(aOpMode);
        robot.setCubeHolderPosition(aOpMode, CUBE_HOLDER_HOLD_POSITION);
        robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_CRYPTO_ALIGN_POSITION);

    }

    public void setTrayFlipToHorizontal(rr_OpMode aOpMode) throws InterruptedException{
        robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_HORIZONTAL_POSITION);
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
            robot.moveRobotToPositionFB(aOpMode, distance, power, isRampedPower);
        } else if (Direction == Backward) {
            robot.moveRobotToPositionFB(aOpMode, -distance, power, isRampedPower);
        } else if (Direction == rr_Constants.DirectionEnum.SidewaysLeft) {
            robot.moveRobotToPositionSideways(aOpMode, distance, power, isRampedPower);
        } else if (Direction == rr_Constants.DirectionEnum.SidewaysRight) {
            robot.moveRobotToPositionSideways(aOpMode, -distance, power, isRampedPower);
        }
    }


}
