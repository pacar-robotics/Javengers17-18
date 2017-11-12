package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Test Cube Arm", group = "Test")

public class TestCubeArm extends rr_OpMode {

    rr_Robot robot;
    float position = 0.5f;
    float orientationPos = .5f;


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
                robot.setCubeClawPosition(0.445f);
                position = 0.445f;
            }

            if (gamepad1.dpad_right) {
                robot.setCubeOrientation(0.9f);
                orientationPos = 1.0f;
            }

            if (gamepad1.dpad_down) {
                robot.setCubeOrientation(0.44f);
                orientationPos = 0.44f;
            }

            processOrientationClaw();
            moveCubeArmWithLimits();

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
            telemetry.addLine("Upper[" +
                    "] Limit Not Pressed");
        }
    }
}
