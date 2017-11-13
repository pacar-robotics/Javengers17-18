package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestJewelArm", group = "TeleOp")
public class TestJewelArm extends rr_OpMode {
    rr_Robot robot;

    float knockerPosition = rr_Constants.JEWEL_PUSHER_NEUTRAL;
    float armPosition = rr_Constants.JEWEL_ARM_UP;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot();

        robot.init(this, this.hardwareMap);

        //robot.setJewelArmPosition(armPosition);
        //robot.setJewelKnockerPosition(knockerPosition);

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processJewelArmTest();
            processJewelKnockerTest();

            telemetry.addLine("Jewel Arm Position: " + armPosition);
            telemetry.addLine("Jewel Knocker Position: " + knockerPosition);

            telemetry.update();

        }

    }

    private void processJewelArmTest() throws InterruptedException {
        if (gamepad1.a) {
            armPosition += .05f;
        } else if (gamepad1.b) {
            armPosition -= .05f;
        }

        robot.setJewelArmPosition(armPosition);
    }

    private void processJewelKnockerTest() throws InterruptedException {
        if (gamepad1.x) {
            knockerPosition += .05f;
        } else if (gamepad1.y) {
            knockerPosition -= .05f;
        }

        //robot.setJewelKnockerPosition(knockerPosition);
    }
}

