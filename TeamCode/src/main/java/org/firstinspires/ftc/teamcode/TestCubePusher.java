package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Cube Pusher", group = "Test")
public class TestCubePusher extends rr_OpMode {
    rr_Robot robot;

    float pusherPosition = 0f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processCubeHolderTest();

            telemetry.addLine("Cube Pusher pos: " + pusherPosition);
            telemetryAddLine("Instruction:"+
                    "Use a & b to calibrate cube aligner");
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setCubePusherPosition(this,pusherPosition);
    }

    private void processCubeHolderTest() throws InterruptedException {
        if (gamepad1.a) {
            pusherPosition += .05f;
        } else if (gamepad1.b) {
            pusherPosition -= .05f;
        }

        robot.setCubePusherPosition(this, pusherPosition);
        Thread.sleep(200);  // Give time for servo to move
    }
}
