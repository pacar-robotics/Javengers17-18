package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic Winch", group = "Test")
public class TestRelicWinch extends rr_OpMode {
    rr_Robot robot;

    float winchPosition = .5f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processWinchTest();

            telemetry.addLine("Winch pos: " + winchPosition);
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);
        robot.setRelicWinchPosition(winchPosition);
    }

    private void processWinchTest() throws InterruptedException {
        if (gamepad1.a) {
            winchPosition += .05f;
        } else if (gamepad1.b) {
            winchPosition -= .05f;
        }

        robot.setRelicWinchPosition(winchPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
