package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_CLAW_INIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_REST;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_UPPER_LIMIT;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WRIST_INIT;

@TeleOp(name = "Test Relic All", group = "Test")
public class TestRelicAll extends rr_OpMode {
    rr_Robot robot;

    float armPosition = RELIC_ARM_INIT;
    float clawPosition = RELIC_CLAW_INIT;
    float wristPosition = RELIC_WRIST_INIT;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processArmTest();
            processClawTest();
            //processRelicWinch();
            processRelicWrist();
            printPositions();

            Thread.sleep(50);  // Give time for servo to move
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
        robot.setRelicArmPosition(armPosition);
        robot.setRelicClawPosition(clawPosition);
        robot.setRelicWristPosition(wristPosition);

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

    private void processClawTest() throws InterruptedException {
        if (gamepad1.x) {
            clawPosition += .05f;
        } else if (gamepad1.y) {
            clawPosition -= .05f;
        }

        robot.setRelicClawPosition(clawPosition);
    }

    private void processRelicWrist() throws InterruptedException {
        if (gamepad1.right_bumper) {
            wristPosition += .05f;
        } else if (gamepad1.left_bumper) {
            wristPosition -= .05f;
        }

        robot.setRelicWristPosition(wristPosition);
        Thread.sleep(200);  // Give time for servo to move
    }

    private void printPositions() {
        telemetry.addLine("Winch pos: " + robot.getRelicWinchPosition());
        telemetry.addLine("Arm pos: " + armPosition);
        telemetry.addLine("Claw pos: " + clawPosition);
        telemetry.addLine("Wrist pos: " + wristPosition);
        telemetry.update();
    }
}
