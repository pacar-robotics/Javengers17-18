package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_SCORING_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.ONE_CUBE_ROW_1;
import static org.firstinspires.ftc.teamcode.rr_Constants.ONE_CUBE_ROW_2;
import static org.firstinspires.ftc.teamcode.rr_Constants.ONE_CUBE_ROW_3;
import static org.firstinspires.ftc.teamcode.rr_Constants.ONE_CUBE_ROW_4;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_RETRACT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_ANGLE_EXTEND;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.SCORING_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.STANDARD_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;

public class rr_TeleLib {

    rr_Robot robot;
    rr_OpMode aOpMode;

    public rr_TeleLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode, aHwMap);
        this.aOpMode = aOpMode;
    }


    //************ PROCESS METHODS ************//

    public void processFieldOrientedDrive() throws InterruptedException {
        //process joysticks

        if (Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(aOpMode,
                    getGamePad2LeftJoystickPolarMagnitude(aOpMode) * SCORING_DRIVE_POWER_FACTOR,
                    getGamePad2LeftJoystickPolarAngle(aOpMode)
                            + 90 - //for rotated orientation of robot at start of game.
                            robot.getMxpGyroSensorHeading(aOpMode)); //for yaw on field.

        } else if (Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * SCORING_DRIVE_POWER_FACTOR;

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else if (Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(aOpMode,
                    getGamePad1LeftJoystickPolarMagnitude(aOpMode) * STANDARD_DRIVE_POWER_FACTOR,
                    getGamePad1LeftJoystickPolarAngle(aOpMode)
                            + 90 - //for rotated orientation of robot at start of game.
                            robot.getMxpGyroSensorHeading(aOpMode)); //for yaw on field.

        } else if (Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad1RightJoystickPolarMagnitude(aOpMode) * STANDARD_DRIVE_POWER_FACTOR;

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else {
            //both joysticks on both gamepads are at rest, stop the robot.
            robot.stopBaseMotors(aOpMode);
        }
    }

    public void processStandardDrive()  throws InterruptedException{
        if (Math.abs(aOpMode.gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(aOpMode,
                    aOpMode.gamepad2.left_stick_x * SCORING_DRIVE_POWER_FACTOR,
                    aOpMode.gamepad2.left_stick_y * SCORING_DRIVE_POWER_FACTOR);

        } else if (Math.abs(aOpMode.gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(aOpMode) * SCORING_DRIVE_POWER_FACTOR;

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else if (Math.abs(aOpMode.gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(aOpMode.gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(aOpMode,
                    aOpMode.gamepad1.left_stick_x * STANDARD_DRIVE_POWER_FACTOR,
                    aOpMode.gamepad1.left_stick_y * STANDARD_DRIVE_POWER_FACTOR);

        } else if (Math.abs(aOpMode.gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad1RightJoystickPolarMagnitude(aOpMode) * STANDARD_DRIVE_POWER_FACTOR;

            if (aOpMode.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runMotors(aOpMode, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runMotors(aOpMode, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            }
        } else {
            //both joysticks on both gamepads are at rest, stop the robot.
            robot.stopBaseMotors(aOpMode);
        }
    }

    public void processCubeArm() throws InterruptedException {

        //**** SCORING (GAMEPAD 2) ****//
        // Manual control takes priority
        if (aOpMode.gamepad2.left_trigger >= TRIGGER_THRESHOLD && !robot.isCubeUpperLimitPressed()) {
            robot.setCubeArmPower(aOpMode, aOpMode.gamepad2.left_trigger);
        } else if (aOpMode.gamepad2.right_trigger >= TRIGGER_THRESHOLD && !robot.isCubeLowerLimitPressed()) {
            robot.setCubeArmPower(aOpMode, -aOpMode.gamepad2.right_trigger);
        } else if (aOpMode.gamepad2.left_bumper) {
            robot.openCubeClawServoOneCube();
        } else if (aOpMode.gamepad2.right_stick_button) {
            robot.setCubeClawToVertical();
        } else if (aOpMode.gamepad2.left_stick_button) {
            robot.setCubeClawToHorizontal();
        } else {

            // Automatic control
            if (aOpMode.gamepad2.x) {
                robot.setCubeClawToHorizontal();
                robot.moveCubeArmToPositionWithLimits(aOpMode, ONE_CUBE_ROW_1, CUBE_ARM_SCORING_POWER);
                robot.openCubeClawServoOneCube();
            } else if (aOpMode.gamepad2.a) {
                robot.moveCubeArmToPositionWithLimits(aOpMode, ONE_CUBE_ROW_2, CUBE_ARM_SCORING_POWER);
                robot.setCubeClawToVertical();
                robot.openCubeClawServoOneCube();
            } else if (aOpMode.gamepad2.y) {
                robot.moveCubeArmToPositionWithLimits(aOpMode, ONE_CUBE_ROW_3, CUBE_ARM_SCORING_POWER);
                robot.setCubeClawToVertical();
                robot.openCubeClawServoOneCube();
            } else if (aOpMode.gamepad2.b) {
                robot.moveCubeArmToPositionWithLimits(aOpMode, ONE_CUBE_ROW_4, CUBE_ARM_SCORING_POWER);
                robot.setCubeClawToVertical();
                robot.openCubeClawServoOneCube();
            } else if (aOpMode.gamepad2.dpad_up) {  // Set claw to intake cubeClawPos
                if (robot.getMotorPosition(aOpMode, CUBE_ARM) == ONE_CUBE_ROW_1) {
                    // Claw needs room to rotate
                    robot.moveCubeArmToPositionWithLimits(aOpMode, ONE_CUBE_ROW_3, CUBE_ARM_SCORING_POWER);
                }
                robot.setCubeClawToHorizontal();
                robot.moveCubeArmToPositionWithLimits(aOpMode, ONE_CUBE_ROW_1, CUBE_ARM_SCORING_POWER);
            }
        }


        //**** COLLECTING (GAMEPAD 1) ****//
        if (aOpMode.gamepad1.right_bumper) {
            robot.openCubeClawServoOneCube();
        } else if (aOpMode.gamepad1.left_bumper) {
            robot.closeCubeClawServoOneCube();
        }
    }

    public void processRelicArm() throws InterruptedException {
        if (aOpMode.gamepad1.x) {
            toggleRelicArm();
        } else if (aOpMode.gamepad1.b) {
            toggleRelicArmAngle();
        } else if (aOpMode.gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
            robot.setPowerExtendRelicArm(aOpMode, aOpMode.gamepad1.left_trigger * RELIC_ARM_EXTEND_POWER_FACTOR);
        } else if (aOpMode.gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            robot.setPowerRetractRelicArm(aOpMode, aOpMode.gamepad1.left_trigger * RELIC_ARM_RETRACT_POWER_FACTOR);
        } else if (aOpMode.gamepad1.right_stick_button) {
            // TODO 17-10-17: Check that lowering and raising methods are not reversed
            robot.setRelicArmAnglePosition(robot.getRelicArmAnglePosition() - 1);
        } else if (aOpMode.gamepad1.left_stick_button) {
            robot.setRelicArmAnglePosition(robot.getRelicArmAnglePosition() + 1);
        }
    }


    //************ PROCESS HELPER METHODS ************//

    public void toggleRelicArm() throws InterruptedException {
        if (robot.getRelicClawPosition() == RELIC_CLAW_OPEN) {
            robot.setRelicClawClosed();
        } else {
            robot.setRelicClawOpen();
        }
    }

    public void toggleRelicArmAngle() throws InterruptedException {
        if (robot.getRelicClawPosition() == RELIC_CLAW_ANGLE_EXTEND) {
            robot.setRelicArmAngleGrab();
        } else {
            robot.setRelicArmAngleExtend();
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
}
