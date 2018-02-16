package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic Wrist", group = "Test")
public class TestRelicWrist extends rr_OpMode {
    rr_Robot robot;

    float wristPosition = 0f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processArmTest();

            telemetry.addLine("Wrist pos: " + wristPosition);
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setRelicWristPosition(wristPosition);
    }

    private void processArmTest() throws InterruptedException {
        if (gamepad1.a) {
            wristPosition += .05f;
        } else if (gamepad1.b) {
            wristPosition -= .05f;
        }

        robot.setRelicWristPosition(wristPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
