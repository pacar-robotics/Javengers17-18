package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic Claw", group = "Test")
public class TestRelicHand extends rr_OpMode {
    rr_Robot robot;

    float handPosition = .5f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processHandTest();

            telemetry.addLine("Hand pos: " + handPosition);
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);
        robot.setRelicHandPosition(handPosition);
    }

    private void processHandTest() throws InterruptedException {
        if (gamepad1.a) {
            handPosition += .05f;
        } else if (gamepad1.b) {
            handPosition -= .05f;
        }

        robot.setRelicWinchPosition(handPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
