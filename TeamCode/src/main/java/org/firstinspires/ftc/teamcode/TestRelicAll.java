package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_REST;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_UPPER_LIMIT;

@TeleOp(name = "Test Relic All", group = "Test")
public class TestRelicAll extends rr_OpMode {
    rr_Robot robot;

    float armPosition = 0f;
    float clawPosition = 0.15f;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processArmTest();
            processClawTest();
            printPositions();

            Thread.sleep(50);  // Give time for servo to move
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setRelicArmPosition(armPosition);
        robot.setRelicClawPosition(clawPosition);
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
        telemetry.addLine("Winch pos: " + robot.getRelicWinchPosition());
        telemetry.addLine("Arm pos: " + armPosition);
        telemetry.addLine("Claw pos: " + clawPosition);
        telemetry.update();
    }
}
