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

    public rr_DiagLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode, aHwMap);
        this.aOpMode = aOpMode;
    }


    //***************** MOTOR TESTS *****************//

    private boolean genericMotorTest(int motor) throws InterruptedException {
        robot.setMotorMode(aOpMode, motor, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorPosition = robot.getMotorPosition(aOpMode, motor);
        robot.testMotor(aOpMode, motor, 0.5f, 1000);    // Runs motor for 1 second at half speed
        int newMotorPosition = robot.getMotorPosition(aOpMode, motor);

        // If the positions are different enough, the motor is running and working
        return !(Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN);
    }

    public boolean testFrontLeftWheel() throws InterruptedException {
        return genericMotorTest(FRONT_LEFT_MOTOR);
    }

    public boolean testFrontRightWheel() throws InterruptedException {
        return genericMotorTest(FRONT_RIGHT_MOTOR);
    }

    public boolean testBackLeftWheel() throws InterruptedException {
        return genericMotorTest(BACK_LEFT_MOTOR);
    }

    public boolean testBackRightWheel() throws InterruptedException {
        return genericMotorTest(BACK_RIGHT_MOTOR);
    }

    public boolean testCubeArmMotor() throws InterruptedException {
        return genericMotorTest(CUBE_ARM);
    }

    public boolean testRelicWinchMotor() throws InterruptedException {
        return genericMotorTest(RELIC_WINCH);
    }


    //***************** PLATFORM TESTS *****************//

    public boolean testForward() throws InterruptedException {
        int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); // Representative motor
        float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); // Save starting angle.
        robot.universalMoveRobot(aOpMode, 0.5f, 0.0f);
        int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
        float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);
        return !((Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3));
    }

    public boolean testLeft() throws InterruptedException {
        int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); // Representative motor
        float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); // Save starting angle.
        robot.universalMoveRobot(aOpMode, 0.0f, 0.5f);
        int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
        float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);
        return !((Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3));
    }
}
