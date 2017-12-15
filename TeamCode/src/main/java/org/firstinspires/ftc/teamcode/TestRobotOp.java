package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ThreadPool;

import static org.firstinspires.ftc.teamcode.rr_Constants.ANALOG_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_LOWER_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_MIDDLE;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_RAISE_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_SAFE_POS;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_ONE_RELEASE;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_TWO_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_EXTEND_IN;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_EXTEND_UP;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_MAX;
import static org.firstinspires.ftc.teamcode.rr_Constants.SCORING_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.STANDARD_DRIVE_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_ONE_CLOSED;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_VERTICAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;

//@TeleOp(name = "Test Robot Op", group = "Test")
public class TestRobotOp extends rr_OpMode {
    rr_Robot robot;

    float cubeClawPos = CUBE_CLAW_OPEN;
    float orientationPos = CUBE_ORIENTATION_HORIZONTAL;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);

        robot.setCubeClawPosition(cubeClawPos);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processClaw();
            processCubeArm();
            processStandardDrive();


            telemetry.addLine("ClawServo: " + cubeClawPos);
            telemetry.addLine("Cube Orientation: " + orientationPos);
            telemetry.addLine("Cube Arm Pos: " + robot.getMotorPosition(this, CUBE_ARM));
            telemetryTouchSensor();
            telemetryAddLine("BRPower" + robot.getMotorPower(this, BACK_RIGHT_MOTOR));
            telemetryAddLine("FRPower" + robot.getMotorPower(this, FRONT_LEFT_MOTOR));
            telemetryAddLine("BLPower" + robot.getMotorPower(this, BACK_LEFT_MOTOR));
            telemetryAddLine("FLPower" + robot.getMotorPower(this, FRONT_LEFT_MOTOR));
            telemetryUpdate();
            processRelicSlide();
            processRelicClaw();
            processRelicHand();

            Thread.sleep(10);
        }
    }


    private void processClaw() throws InterruptedException {
        if (gamepad1.dpad_right && cubeClawPos < .9) {
            cubeClawPos += .025f;
            robot.setCubeClawPosition(cubeClawPos);
            Thread.sleep(250);
        } else if (gamepad1.dpad_down && cubeClawPos > 0) {
            cubeClawPos -= .025f;
            robot.setCubeClawPosition(cubeClawPos);
            Thread.sleep(250);
        }

        if (gamepad1.right_bumper && cubeClawPos == CUBE_CLAW_OPEN) {
            robot.closeCubeClawServoOneCube();
            cubeClawPos = CUBE_CLAW_ONE_CLOSED;
            Thread.sleep(200);
        } else if (gamepad1.right_bumper) {
            robot.openCubeClawServoOneCube();
            cubeClawPos = CUBE_CLAW_OPEN;
            Thread.sleep(200);
        }
    }


    private void processCubeArm() throws InterruptedException{
        if (gamepad1.left_trigger >= TRIGGER_THRESHOLD && !robot.isCubeLowerLimitPressed()) {
            robot.setCubeArmPower(this, 0.1f);
        } else if (gamepad1.right_trigger >= TRIGGER_THRESHOLD && !robot.isCubeUpperLimitPressed()) {
            robot.setCubeArmPower(this, -0.4f);
        } else {
            robot.setCubeArmPower(this, 0.0f);
        }

        if (gamepad1.a) {
            robot.openCubeClawServoOneCube();
            robot.moveCubeArmToPositionWithTouchLimits(this, CUBE_ARM_GRAB, CUBE_ARM_RAISE_POWER); //TODO: CHANGE CUBE_ARM_GRAB
        }
        if (gamepad1.y) {
            robot.closeCubeClawServoOneCube();
            robot.moveCubeArmToPositionWithTouchLimits(this, CUBE_ARM_MIDDLE, CUBE_ARM_RAISE_POWER); //TODO: CHANGE CUBE_ARM_MIDDLE
        }
        if (gamepad1.b) {
            robot.setCubeClawPosition(CUBE_CLAW_ONE_RELEASE);
            robot.moveRobotToPositionFB(this, -6, 0.25f, false);
            robot.closeCubeClawServoOneCube();
            cubeClawPos = CUBE_CLAW_ONE_CLOSED;
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
        if (Math.abs(gamepad2.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad2RightJoystickPolarMagnitude(this) * SCORING_DRIVE_POWER_FACTOR;

            if (this.gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runRampedMotors(this, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runRampedMotors(this, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            }
        } else if (Math.abs(gamepad2.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(gamepad2.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(this,
                    gamepad2.left_stick_x * SCORING_DRIVE_POWER_FACTOR,
                    gamepad2.left_stick_y * SCORING_DRIVE_POWER_FACTOR);

        } else if (Math.abs(gamepad1.right_stick_x) > ANALOG_STICK_THRESHOLD) {

            //we are not in deadzone. Driver is pushing right joystick, sideways
            float turnVelocity = (float) getGamePad1RightJoystickPolarMagnitude(this) * TURN_POWER_FACTOR;

            if (gamepad1.right_stick_x > 0) {
                //turn clockwise to correct magnitude
                robot.runRampedMotors(this, -turnVelocity, turnVelocity, -turnVelocity, turnVelocity);
            } else {
                //turn counter-clockwise
                robot.runRampedMotors(this, turnVelocity, -turnVelocity, turnVelocity, -turnVelocity);
            }
        } else if (Math.abs(gamepad1.left_stick_x) > ANALOG_STICK_THRESHOLD ||
                Math.abs(gamepad1.left_stick_y) > ANALOG_STICK_THRESHOLD) {
            //we are not in deadzone. Driver is pushing left joystick
            //lets make the robot move in chosen angle and magnitude.
            robot.universalMoveRobot(this,
                    gamepad1.left_stick_x * STANDARD_DRIVE_POWER_FACTOR,
                    gamepad1.left_stick_y * STANDARD_DRIVE_POWER_FACTOR);

        }  else {
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

    public void processRelicSlide(){
        if(Math.abs(gamepad2.right_trigger)>TRIGGER_THRESHOLD){
            robot.setRelicWinchPower(0.25f);
        }else if(Math.abs(gamepad2.left_trigger)>TRIGGER_THRESHOLD){
            robot.setRelicWinchPower(-0.25f);
        }else{
            //the triggers are in dead zone.
            //stop the relic slide
            robot.setRelicWinchPower(0);
        }
    }

    public void processRelicClaw() throws InterruptedException{
        if(gamepad2.x){
            robot.setRelicClawClosed();
        }
        if(gamepad2.b){
            robot.setRelicClawOpen();
        }
    }
    public void processRelicHand() throws InterruptedException{
        if(gamepad2.a){
            robot.setRelicArmPosition(RELIC_ARM_GRAB);
        }
        if(gamepad2.left_bumper){
            robot.setRelicArmPosition(RELIC_ARM_EXTEND_IN);
        }
        if(gamepad2.right_bumper) {
            robot.setRelicArmPosition(RELIC_ARM_EXTEND_UP);
        }

    }

}
