package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_REST;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_UPPER_LIMIT;

@TeleOp(name = "Test Relic All", group = "Test")
public class TestRelicAll extends rr_OpMode {
    rr_Robot robot;

    float armPosition = 0f;
    float clawPosition = 0.15f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processWinchTest();
            processArmTest();
            processClawTest();
            printPositions();

            Thread.sleep(50);  // Give time for servo to move
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setRelicArmPosition(armPosition);
        robot.setRelicClawPosition(clawPosition);
    }

    private void processWinchTest() throws InterruptedException {
        if (gamepad1.right_trigger >= .05f) {
            robot.setRelicWinchPower(gamepad1.right_trigger * RELIC_WINCH_EXTEND_POWER_FACTOR);
        } else if (gamepad1.left_trigger >= .05f) {
            robot.setRelicWinchPower(-gamepad1.left_trigger * RELIC_WINCH_RETRACT_POWER_FACTOR);
        } else {
            robot.setRelicWinchPower(0);
        }

        if (gamepad1.dpad_up) {
            robot.setRelicWinchPosition(this, RELIC_WINCH_UPPER_LIMIT, 50);
        } else if (gamepad1.dpad_down) {
            robot.setRelicWinchPosition(this, RELIC_WINCH_REST, 50);
        }
    }

    private void processArmTest() throws InterruptedException {
        if (gamepad1.x) {
            armPosition += .05f;
        } else if (gamepad1.y) {
            armPosition -= .05f;
        }

        robot.setRelicArmPosition(armPosition);
    }

    private void processClawTest() throws InterruptedException {
        if (gamepad1.a) {
            clawPosition += .05f;
        } else if (gamepad1.b) {
            clawPosition -= .05f;
        }

        robot.setRelicClawPosition(clawPosition);
    }

    private void printPositions() {
        telemetry.addLine("Winch pos: " + robot.getRelicWinchPosition());
        telemetry.addLine("Arm pos: " + armPosition);
        telemetry.addLine("Claw pos: " + clawPosition);
        telemetry.update();
    }
}
