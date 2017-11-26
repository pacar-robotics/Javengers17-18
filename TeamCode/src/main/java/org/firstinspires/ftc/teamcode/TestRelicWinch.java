package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;

@TeleOp(name = "Test Relic Winch", group = "Test")
public class TestRelicWinch extends rr_OpMode {
    rr_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processWinchTest();

            telemetry.addLine("Winch pos: " + robot.getRelicWinchPosition());
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
    }

    private void processWinchTest() throws InterruptedException {
        if (gamepad1.right_trigger >= .05f) {
            robot.setRelicWinchPower(gamepad1.right_trigger * RELIC_WINCH_EXTEND_POWER_FACTOR);
        } else if (gamepad1.left_trigger >= .05f) {
            robot.setRelicWinchPower(-gamepad1.left_trigger * RELIC_WINCH_RETRACT_POWER_FACTOR);
        } else {
            robot.setRelicWinchPower(0);
        }
    }
}
