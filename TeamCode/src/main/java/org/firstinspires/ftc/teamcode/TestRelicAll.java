package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic All", group = "Test")
public class TestRelicAll extends rr_OpMode {
    rr_Robot robot;

    float winchPosition = 0f;
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
        robot = new rr_Robot(this, this.hardwareMap);
        robot.setRelicWinchPosition(winchPosition);
        robot.setRelicArmPosition(armPosition);
        robot.setRelicClawPosition(clawPosition);
    }

    private void processWinchTest() throws InterruptedException {
        if (gamepad1.left_bumper) {
            winchPosition += .05f;
        } else if (gamepad1.right_bumper) {
            winchPosition -= .05f;
        }

        robot.setRelicWinchPosition(winchPosition);
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
        telemetry.addLine("Winch pos: " + winchPosition);
        telemetry.addLine("Arm pos: " + armPosition);
        telemetry.addLine("Claw pos: " + clawPosition);
        telemetry.update();
    }
}
