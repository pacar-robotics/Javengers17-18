package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_HIGH;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_EXTEND_POWER_FACTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_RETRACT_POWER_FACTOR;

@TeleOp(name = "Test Cube CounterRotation", group = "Test")
public class TestCubeCounterRotation extends rr_OpMode {
    rr_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            processCounterRotation();

            telemetry.addLine("Optical Range: " +
                    robot.getIntakeOpticalRightSensorRange(this));
            telemetry.addLine("UltraSonic Range: " +
                    robot.getIntakeUltrasonicRightSensorRange(this));
            telemetry.update();
        }
    }

    private void initialize() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);
    }

    private void processCounterRotation() throws InterruptedException {
       double  opticalRange=robot.getIntakeOpticalRightSensorRange(this);
       double  ultrasonicRange=robot.getIntakeUltrasonicRightSensorRange(this);

        if (
                (opticalRange > 4.5)
                        && (ultrasonicRange <4)
                )
        {
            //the cube is going in sideways
            //we should switch to low power but reverse one of the motors
            //this should cause the motors to rotate the cube so it is straight
            //this has to be tested and adjusted.
            robot.setIntakePower(this, INTAKE_POWER_HIGH, -INTAKE_POWER_HIGH);
            Thread.sleep(200); //wait for a little time for rotation.
            robot.setIntakePower(this, 0, 0);
            Thread.sleep(100); //wait for a second for rotation.
        } else

        {
            robot.setIntakePower(this, INTAKE_POWER_HIGH, INTAKE_POWER_HIGH);
        }
    }
}
