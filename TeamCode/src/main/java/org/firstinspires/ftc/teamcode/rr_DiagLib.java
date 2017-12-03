package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH;

public class rr_DiagLib {

    private final static float MECCANUM_WHEEL_ENCODER_MARGIN = 50;

    rr_Robot robot;
    rr_OpMode aOpMode;

    private class RobotTest {
        private String elementName;
        private boolean testResult;
        private String testMessage;

        public RobotTest(String elementName, boolean testResult) {
            this.elementName = elementName;
            this.testResult = testResult;
        }

        public RobotTest(String elementName, boolean testResult, String testMessage) {
            this.elementName = elementName;
            this.testResult = testResult;
            this.testMessage = testMessage;
        }
    }

    public rr_DiagLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode);
        robot.autonomousInit(aOpMode, aHwMap);
        this.aOpMode = aOpMode;
    }


    //***************** MOTOR TESTS *****************//

    private RobotTest genericMotorTest(int motor, String motorName) throws InterruptedException {
        robot.setMotorMode(aOpMode, motor, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorPosition = robot.getMotorPosition(aOpMode, motor);
        robot.testMotor(aOpMode, motor, 0.5f, 1000);    // Runs motor for 1 second at half speed
        int newMotorPosition = robot.getMotorPosition(aOpMode, motor);

        // If the positions are different enough, the motor is running and working
        if (Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) {
            return new RobotTest(motorName, true);
        } else {
            return new RobotTest(motorName, false, "Failed to detect rotation");
        }
    }

    public RobotTest testFrontLeftWheel() throws InterruptedException {
        return genericMotorTest(FRONT_LEFT_MOTOR, "Front left wheel");
    }

    public RobotTest testFrontRightWheel() throws InterruptedException {
        return genericMotorTest(FRONT_RIGHT_MOTOR, "Front right wheel");
    }

    public RobotTest testBackLeftWheel() throws InterruptedException {
        return genericMotorTest(BACK_LEFT_MOTOR, "Back left wheel");
    }

    public RobotTest testBackRightWheel() throws InterruptedException {
        return genericMotorTest(BACK_RIGHT_MOTOR, "Back right wheel");
    }

    public RobotTest testCubeArmMotor() throws InterruptedException {
        return genericMotorTest(CUBE_ARM, "Cube arm motor");
    }

    public RobotTest testRelicWinchMotor() throws InterruptedException {
        return genericMotorTest(RELIC_WINCH, "Relic winch motor");
    }


    //***************** PLATFORM TESTS *****************//

    public RobotTest testForward() throws InterruptedException {
        int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); // Representative motor
        float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); // Save starting angle.
        robot.universalMoveRobot(aOpMode, 0.5f, 0.0f);
        int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
        float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);

        if ((Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
            return new RobotTest("Platform forwards/backwards", true);
        } else {
            return new RobotTest("Platform forwards/backwards", false, "Failed to detect platform movement");
        }
    }

    public RobotTest testLeft() throws InterruptedException {
        int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); // Representative motor
        float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); // Save starting angle.
        robot.universalMoveRobot(aOpMode, 0.0f, 0.5f);
        int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
        float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);

        if ((Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
            return new RobotTest("Platform sideways", true);
        } else {
            return new RobotTest("Platform sideways", true, "Failed to detect platform movement");
        }
    }
}
