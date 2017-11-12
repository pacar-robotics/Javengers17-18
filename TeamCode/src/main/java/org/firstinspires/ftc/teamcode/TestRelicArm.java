package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic Arm", group = "Test")
public class TestRelicArm extends rr_OpMode {
    rr_Robot robot;

    float armPosition = .5f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processArmTest();

            telemetry.addLine("Arm pos: " + armPosition);
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);
        robot.setRelicArmPosition(armPosition);
    }

    private void processArmTest() throws InterruptedException {
        if (gamepad1.a) {
            armPosition += .05f;
        } else if (gamepad1.b) {
            armPosition -= .05f;
        }

        robot.setRelicArmPosition(armPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
