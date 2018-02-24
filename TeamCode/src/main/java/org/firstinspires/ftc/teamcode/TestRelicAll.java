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
    float wristPosition = 0f;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processArmTest();
            processClawTest();
            processRelicWinch();
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
        if (gamepad1.a) {
            clawPosition += .05f;
        } else if (gamepad1.b) {
            clawPosition -= .05f;
        }

        robot.setRelicClawPosition(clawPosition);
    }

    private void processRelicWinch() throws InterruptedException {
        if (gamepad1.a) {
            clawPosition += .05f;
        } else if (gamepad1.b) {
            clawPosition -= .05f;
        }

        robot.setRelicClawPosition(clawPosition);
        Thread.sleep(200);  // Give time for servo to move
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
