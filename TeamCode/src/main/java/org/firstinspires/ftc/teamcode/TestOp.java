package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.SCORING_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.STANDARD_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_ONE_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_VERTICAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;

@TeleOp(name = "Test Op", group = "Test")
public class TestOp extends rr_OpMode {
    rr_Robot robot;
    float position = CUBE_CLAW_OPEN;
    float orientationPos = CUBE_ORIENTATION_HORIZONTAL;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);

        robot.setCubeClawPosition(position);
        robot.setCubeOrientation(orientationPos);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                closeClaw();
                Thread.sleep(250);
            }
            if (gamepad1.b) {
                openClaw();
                Thread.sleep(250);
            }

            if (gamepad1.right_bumper) {
                robot.setCubeClawPosition(CUBE_CLAW_ONE_CLOSED);
                position = CUBE_CLAW_ONE_CLOSED;
            }

            if (gamepad1.left_bumper) {
                robot.setCubeClawPosition(CUBE_CLAW_OPEN);
                position = CUBE_CLAW_OPEN;
            }

            if (gamepad1.dpad_right) {
                robot.setCubeOrientation(CUBE_ORIENTATION_HORIZONTAL);
                orientationPos = CUBE_ORIENTATION_HORIZONTAL;
            }

            if (gamepad1.dpad_up) {
                robot.setCubeOrientation(CUBE_ORIENTATION_VERTICAL);
                orientationPos = CUBE_ORIENTATION_VERTICAL;
            }

            processOrientationClaw();
            moveCubeArmWithLimits();
            processStandardDrive();

            telemetry.addLine("ClawServo: " + position);
            telemetry.addLine("Cube Orientation: " + orientationPos);
            telemetry.addLine("Cube Arm Pos: " + robot.getMotorPosition(this, CUBE_ARM));
            telemetryTouchSensor();
            telemetryUpdate();

            Thread.sleep(10);
        }
    }

    private void processOrientationClaw() throws InterruptedException {
        if (gamepad1.x && orientationPos < .9) {
            orientationPos += .05f;
            robot.setCubeOrientation(orientationPos);
            Thread.sleep(250);
        } else if (gamepad1.y && orientationPos > 0) {
            orientationPos -= .05f;
            robot.setCubeOrientation(orientationPos);
            Thread.sleep(250);
        }
    }

    private void openClaw() throws InterruptedException {
        if (position < 0.95) {
            robot.setCubeClawPosition(Math.abs(position + .05f));
            position = position + .025f;
        }
    }

    private void closeClaw() throws InterruptedException {
        if (position > 0.05) {
            robot.setCubeClawPosition(Math.abs(position - .05f));
            position = position - .025f;
        }
    }

    private void moveCubeArmWithLimits() {
        if (gamepad1.left_trigger >= TRIGGER_THRESHOLD && !robot.isCubeLowerLimitPressed()) {
            robot.setCubeArmPower(this, 0.1f);
        } else if (gamepad1.right_trigger >= TRIGGER_THRESHOLD && !robot.isCubeUpperLimitPressed()) {
            robot.setCubeArmPower(this, -0.4f);
        } else {
            robot.setCubeArmPower(this, 0.0f);
        }
    }

    private void moveCubeArmWithoutLimits() {
        if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
            robot.setCubeArmPower(this, 0.25f);
        } else if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            robot.setCubeArmPower(this, -0.15f);
        } else {
            robot.setCubeArmPower(this, 0.0f);
        }
    }

    private void telemetryTouchSensor() {
        if (robot.isCubeLowerLimitPressed()) {
            telemetry.addLine("Lower Limit Pressed");
        } else {
            telemetry.addLine("Lower Limit Not Pressed");
        }
        if (robot.isCubeUpperLimitPressed()) {
            telemetry.addLine("Upper Limit Pressed");
        } else {
            telemetry.addLine("Upper" +
                    " Limit Not Pressed");
        }
    }

    public void processStandardDrive()  throws InterruptedException{
        if (Math.abs(gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(this,
                    gamepad2.left_stick_x * SCORING_DRIVE_POWER_FACTOR,
                    gamepad2.left_stick_y * SCORING_DRIVE_POWER_FACTOR);

        } else if (Math.abs(gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(this) * SCORING_DRIVE_POWER_FACTOR;

            if (this.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runMotors(this, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runMotors(this, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            }
        } else if (Math.abs(gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(this,
                    gamepad1.left_stick_x * STANDARD_DRIVE_POWER_FACTOR,
                    gamepad1.left_stick_y * STANDARD_DRIVE_POWER_FACTOR);

        } else if (Math.abs(gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad1RightJoystickPolarMagnitude(this) * TURN_POWER_FACTOR;

            if (gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runMotors(this, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runMotors(this, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            }
        } else {
            //both joysticks on both gamepads are at rest, stop the robot.
            robot.stopBaseMotors(this);
        }
    }

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
}
