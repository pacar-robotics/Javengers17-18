package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestJewelArm", group = "Test")
public class  TestJewelArm extends rr_OpMode {
    rr_Robot robot;

    float knockerPosition = rr_Constants.JEWEL_PUSHER_NEUTRAL;
    float armPosition = rr_Constants.JEWEL_ARM_UP;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);

        robot.setJewelArmPosition(armPosition);
        robot.setJewelPusherPosition(knockerPosition);

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processJewelArmTest();
            processJewelKnockerTest();

            telemetry.addLine("Jewel Arm Position: " + armPosition);
            telemetry.addLine("Jewel Knocker Position: " + knockerPosition);

            processColorSensorsTest();

            telemetry.update();

        }

    }

    private void processJewelArmTest() throws InterruptedException {
        if (gamepad1.a) {
            armPosition += .01f;
        } else if (gamepad1.b) {
            armPosition -= .01f;
        }

        robot.setJewelArmPosition(armPosition);
    }

    private void processJewelKnockerTest() throws InterruptedException {
        if (gamepad1.x) {
            knockerPosition += .01f;
        } else if (gamepad1.y) {
            knockerPosition -= .01f;
        }

        robot.setJewelPusherPosition(knockerPosition);
    }

    private void processColorSensorsTest() throws InterruptedException {
        telemetryAddLine("Left Jewel Color:"+robot.getJewelLeftColor(this).toString());
        telemetryAddLine("Left Luminosity:"+robot.getJewelLeftLumunosity(this));

        telemetryAddLine("Right Jewel Color:" + robot.getJewelRightColor(this).toString());
        telemetryAddLine("Right Luminosity:"+robot.getJewelRightLumunosity(this));

        telemetryAddLine("Right Jewel Distance:" + robot.getRightJewelRange(this));
        telemetryAddLine("Left Jewel Distance:" + robot.getLeftJewelRange(this));

        telemetry.update();

    }
}

