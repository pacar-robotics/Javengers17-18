package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Calibrate Cube Arm", group = "Test")

public class TestCubeArm extends rr_OpMode {

    rr_Robot robot;
    float position = 0.5f;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);

        robot.setCubeClawPosition(position);

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

            moveCubeArmWithoutLimits();

            telemetry.addLine("ClawServo: " + robot.getCubeClawPosition());
            telemetry.addLine("CubeArm: " + position);
            telemetryTouchSensor();
            telemetryUpdate();

            Thread.sleep(10);
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
        if (gamepad2.left_trigger >= TRIGGER_THRESHOLD && !robot.isCubeUpperLimitPressed()) {
            robot.setCubeArmPower(this, gamepad2.left_trigger);
        } else if (gamepad2.right_trigger >= TRIGGER_THRESHOLD && !robot.isCubeLowerLimitPressed()) {
            robot.setCubeArmPower(this, -gamepad2.right_trigger);
        }
    }

    private void moveCubeArmWithoutLimits() {
        if (gamepad2.left_trigger >= TRIGGER_THRESHOLD) {
            robot.setCubeArmPower(this, gamepad2.left_trigger);
        } else if (gamepad2.right_trigger >= TRIGGER_THRESHOLD) {
            robot.setCubeArmPower(this, -gamepad2.right_trigger);
        }
    }

    private void telemetryTouchSensor() {
        if (!robot.isCubeLowerLimitPressed()) {
            telemetry.addLine("Lower Limit Pressed");
        } else {
            telemetry.addLine("Lower Limit Not Pressed");
        }
        if (!robot.isCubeUpperLimitPressed()) {
            telemetry.addLine("Upper Limit Pressed");
        } else {
            telemetry.addLine("Upper[" +
                    "] Limit Not Pressed");
        }
    }
}
