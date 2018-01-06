package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_HORIZONTAL;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_ONE_RELEASE;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Test Cube Arm", group = "Test")

public class TestCubeArm extends rr_OpMode {

    rr_Robot robot;

    
    float cubeClawPosition = CUBE_CLAW_OPEN;
    float cubeArmPosition = CUBE_ARM_GRAB;
    float position = CUBE_CLAW_ONE_RELEASE;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);

        robot.setCubeClawPosition(cubeClawPosition);

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
                cubeClawPosition = 0.445f;
            }


            moveCubeArmWithLimits();

            telemetry.addLine("ClawServo: " + cubeClawPosition);
            telemetry.addLine("Cube Arm Pos: " + robot.getMotorPosition(this, CUBE_ARM));
            telemetryTouchSensor();
            telemetryUpdate();

            Thread.sleep(10);
        }
    }

    private void openClaw() throws InterruptedException {
        if (cubeClawPosition < 0.95) {
            robot.setCubeClawPosition(Math.abs(cubeClawPosition + .05f));
            cubeClawPosition = cubeClawPosition + .025f;
        }
    }

    private void closeClaw() throws InterruptedException {
        if (cubeClawPosition > 0.05) {
            robot.setCubeClawPosition(Math.abs(cubeClawPosition - .05f));
            cubeClawPosition = cubeClawPosition - .025f;
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
