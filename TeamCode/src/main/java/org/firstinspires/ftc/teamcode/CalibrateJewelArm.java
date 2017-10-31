package org.firstinspires.ftc.teamcode;

public class CalibrateJewelArm extends rr_OpMode {
    rr_Robot robot;

    float knockerPosition = 0.5f;
    float armPosition = 0.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);

        robot.setJewelArmPosition(armPosition);
        robot.setJewelKnockerPosition(knockerPosition);

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processJewelArmTest();
            processJewelKnockerTest();

            telemetry.addLine("Jewel Arm Position: " + armPosition);
            telemetry.addLine("Jewel Knocker Position: " + knockerPosition);

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

        robot.setJewelKnockerPosition(knockerPosition);
    }
}
