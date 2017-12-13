package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic Claw", group = "Test")
public class TestRelicClaw extends rr_OpMode {
    rr_Robot robot;

    float clawPosition = 0.15f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processClawTest();

            telemetry.addLine("Claw pos: " + clawPosition);
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setRelicClawPosition(clawPosition);
    }

    private void processClawTest() throws InterruptedException {
        if (gamepad1.a) {
            clawPosition += .05f;
        } else if (gamepad1.b) {
            clawPosition -= .05f;
        }

        robot.setRelicClawPosition(clawPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
