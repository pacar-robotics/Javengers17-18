package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Relic Arm", group = "Test")
public class TestRelicArm extends rr_OpMode {
    rr_Robot robot;

    float winchPosition = .5f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processWinchTest();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);
        robot.setRelicWinchPosition(winchPosition);
    }

    private void processWinchTest() {
        if (gamepad1.a) {
            winchPosition += .5f;
        } else if (gamepad1.b) {
            winchPosition -= .5f;
        }

        robot.setRelicWinchPosition(winchPosition);
    }
}
