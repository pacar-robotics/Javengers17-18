package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM_GRAB;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ORIENTATION_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "Calibrate Cube Arm", group = "Calibrations")

public class CalibrateCubeArm extends rr_OpMode {

    rr_Robot robot;
    float cubeClawPosition = CUBE_CLAW_OPEN;
    float cubeOrientationPosition = CUBE_ORIENTATION_HORIZONTAL;
    float cubeArmPosition=CUBE_ARM_GRAB;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);

        robot.setCubeClawPosition(CUBE_CLAW_OPEN); //standard claw open for one cube.
        robot.setCubeOrientation(CUBE_ORIENTATION_HORIZONTAL);//oriented horizontal

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            //process and display settings for claw orientation and claw position
            if (gamepad1.b && cubeClawPosition<0.9) {
                //tighten the claw
                robot.setCubeClawPosition(cubeClawPosition+=0.05);
                Thread.sleep(250);
            }
            if (gamepad1.x && cubeClawPosition>0.1) {
                //open the claw
                robot.setCubeClawPosition(cubeClawPosition-=0.05);
                Thread.sleep(250);
            }

            if (gamepad1.y && cubeClawPosition<0.9) {
                //rotate the claw orientation by positive increments
                robot.setCubeOrientation(cubeOrientationPosition+=0.05);
                Thread.sleep(250);
            }

            if (gamepad1.a && cubeClawPosition>0.1) {
                //rotate the claw orientation by negative increments
                robot.setCubeOrientation(cubeOrientationPosition-=0.05);
                Thread.sleep(250);
            }



            moveCubeArmWithLimits(); //will process


            DBG("Claw Pos:"+cubeClawPosition);
            DBG("Orientation Pos:"+cubeOrientationPosition);
            DBG("Cube Arm Pos:"+robot.getMotorPosition(this, CUBE_ARM));


            telemetryUpdate();

            Thread.sleep(10);
        }
    }

    private void processOrientationClaw() throws InterruptedException {

    }



    private void moveCubeArmWithLimits() {
        if (gamepad1.left_trigger >= TRIGGER_THRESHOLD && !robot.isCubeLowerLimitPressed()) {
            robot.setCubeArmPower(this, 0.1f);
        } else if (gamepad1.right_trigger >= TRIGGER_THRESHOLD && !robot.isCubeUpperLimitPressed()) {
            robot.setCubeArmPower(this, -0.4f);
        } else {
            robot.setCubeArmPower(this, 0.0f);
        }
    }

    private void moveCubeArmWithoutLimits() {
        if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
            robot.setCubeArmPower(this, 0.25f);
        } else if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            robot.setCubeArmPower(this, -0.15f);
        } else {
            robot.setCubeArmPower(this, 0.0f);
        }
    }

    private void telemetryTouchSensor() {
        if (robot.isCubeLowerLimitPressed()) {
            telemetry.addLine("Lower Limit Pressed");
        } else {
            telemetry.addLine("Lower Limit Not Pressed");
        }
        if (robot.isCubeUpperLimitPressed()) {
            telemetry.addLine("Upper Limit Pressed");
        } else {
            telemetry.addLine("Upper[" +
                    "] Limit Not Pressed");
        }
    }
}
