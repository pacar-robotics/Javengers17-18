package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Cube Holder", group = "Test")
public class TestCubeHolder extends rr_OpMode {
    rr_Robot robot;

    float holderPosition = 0f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processCubeHolderTest();

            telemetry.addLine("Cube Holder pos: " + holderPosition);
            telemetryAddLine("Instruction:"+
                    "Use a & b to calibrate cube holder");
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setCubeHolderPosition(this, holderPosition);
    }

    private void processCubeHolderTest() throws InterruptedException {
        if (gamepad1.a) {
            holderPosition += .05f;
        } else if (gamepad1.b) {
            holderPosition -= .05f;
        }

        robot.setCubeHolderPosition(this, holderPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
