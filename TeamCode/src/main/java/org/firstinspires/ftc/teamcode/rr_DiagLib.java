package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_EXTEND_UP;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_MOTOR;

class rr_DiagLib {

    // Used for motor and platform movements
    private final static float MECCANUM_WHEEL_ENCODER_MARGIN = 50;
    private static final int SERVO_WAIT_TIME = 250;     // milliseconds

    private rr_Robot robot;
    private rr_OpMode aOpMode;
    ArrayList<RobotTest> robotTests;

    rr_DiagLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode);
        robot.autonomousInit(aOpMode, aHwMap);  // teleOpInit ignores some sensors
        this.aOpMode = aOpMode;

        robotTests = new ArrayList<>();
        initializeRobotTests();
    }


    //***************** INNER CLASSES *****************//

    enum TestType {AUTOMATIC, MANUAL}

    interface AutomaticTest {
        // Can test themselves and say what went wrong
        TestResult runTest() throws InterruptedException;
    }

    interface ManualTest {
        // Can run themselves but require a user to assess the test
        void runTest() throws InterruptedException;
    }

    class RobotTest {
        private String testName;
        private TestType testType;
        private AutomaticTest automaticTest;
        private ManualTest manualTest;

        RobotTest(String testName, TestType testType, AutomaticTest automaticTest) {
            this.testName = testName;
            this.testType = testType;
            this.automaticTest = automaticTest;
        }

        RobotTest(String testName, TestType testType, ManualTest manualTest) {
            this.testName = testName;
            this.testType = testType;
            this.manualTest = manualTest;
        }

        TestType getTestType() {
            return testType;
        }

        AutomaticTest getAutomaticTest() {
            return automaticTest;
        }

        ManualTest getManualTest() {
            return manualTest;
        }

        String getTestName() {
            return testName;
        }
    }

    class TestResult {
        private String elementName;
        private boolean testResult;
        private String testMessage;

        TestResult(String elementName, boolean testResult) {
            this.elementName = elementName;
            this.testResult = testResult;
        }

        TestResult(String elementName, boolean testResult, String testMessage) {
            this.elementName = elementName;
            this.testResult = testResult;
            this.testMessage = testMessage;
        }

        String getElementName() {
            return elementName;
        }

        boolean getTestResult() {
            return testResult;
        }

        String getTestMessage() {
            return testMessage;
        }
    }


    //***************** CLASS METHODS *****************//

    private void initializeRobotTests() {
        robotTests.add(new RobotTest("Front Left Wheel", TestType.AUTOMATIC, new TestFrontLeftWheel()));
        robotTests.add(new RobotTest("Front Right Wheel", TestType.AUTOMATIC, new TestFrontRightWheel()));
        robotTests.add(new RobotTest("Back Left Wheel", TestType.AUTOMATIC, new TestBackLeftWheel()));
        robotTests.add(new RobotTest("Back Right Wheel", TestType.AUTOMATIC, new TestBackRightWheel()));
        robotTests.add(new RobotTest("Tray Lift Motor", TestType.AUTOMATIC, new TestTrayLiftMotor()));
        robotTests.add(new RobotTest("Relic Winch Motor", TestType.AUTOMATIC, new TestRelicWinchMotor()));

        robotTests.add(new RobotTest("Platform Forward", TestType.AUTOMATIC, new TestPlatformForward()));
        robotTests.add(new RobotTest("Platform Left", TestType.AUTOMATIC, new TestPlatformLeft()));
        robotTests.add(new RobotTest("Platform Diagonal", TestType.AUTOMATIC, new TestPlatformDiagonal()));


        robotTests.add(new RobotTest("Intake Motors", TestType.MANUAL, new TestIntakeMotors()));
        robotTests.add(new RobotTest("Relic Arm", TestType.MANUAL, new TestRelicArm()));
        robotTests.add(new RobotTest("Relic Claw", TestType.MANUAL, new TestRelicClaw()));
        robotTests.add(new RobotTest("Cube Pusher", TestType.MANUAL, new TestCubePusher()));
        robotTests.add(new RobotTest("Tray Flipper", TestType.MANUAL, new TestTrayFlipper()));
        robotTests.add(new RobotTest("Jewel Arm", TestType.MANUAL, new TestJewelArm()));

        robotTests.add(new RobotTest("Test Connection", TestType.MANUAL, new TestConnection()));
    }


    //***************** MOTOR TESTS *****************//

    private TestResult genericMotorTest(int motor, String motorName, boolean defaultDirection) throws InterruptedException {
        robot.setMotorMode(aOpMode, motor, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorPosition = robot.getMotorPosition(aOpMode, motor);
        // Runs motor for 1 second at half speed
        robot.testMotor(aOpMode, motor, 0.5f * (defaultDirection ? 1 : -1), 1000);
        int newMotorPosition = robot.getMotorPosition(aOpMode, motor);

        // Move motor back if requested
        robot.testMotor(aOpMode, motor, -0.5f * (defaultDirection ? 1 : -1), 1000);

        // If the positions are different enough, the motor and encoder are working
        if (Math.abs(newMotorPosition - motorPosition) > MECCANUM_WHEEL_ENCODER_MARGIN) {
            return new TestResult(motorName, true);
        } else {
            return new TestResult(motorName, false, "Motor not moving or faulty encoder");
        }
    }

    private class TestFrontLeftWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(FRONT_LEFT_MOTOR, "Front left wheel", true);
        }
    }

    private class TestFrontRightWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(FRONT_RIGHT_MOTOR, "Front right wheel", true);
        }
    }

    private class TestBackLeftWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(BACK_LEFT_MOTOR, "Back left wheel", true);
        }
    }

    private class TestBackRightWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(BACK_RIGHT_MOTOR, "Back right wheel", true);
        }
    }

    private class TestTrayLiftMotor implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(TRAY_LIFT_MOTOR, "Tray lift motor", false);
        }
    }

    private class TestRelicWinchMotor implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(RELIC_WINCH_MOTOR, "Relic winch motor", true);
        }
    }

    private class TestIntakeMotors implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.testMotor(aOpMode, INTAKE_RIGHT_MOTOR, 0.5f, 1000);
            robot.testMotor(aOpMode, INTAKE_LEFT_MOTOR, 0.5f, 1000);
        }
    }


    //***************** PLATFORM TESTS *****************//

    private TestResult genericPlatformTest(String platformName, float xVelocity, float yVelocity) throws InterruptedException {
        int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);  // Representative motor
        float startingAngle = robot.getBoschGyroSensorHeading(aOpMode);

        robot.universalMoveRobot(aOpMode, xVelocity, yVelocity);
        Thread.sleep(1000);
        robot.stopBaseMotors(aOpMode);

        int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
        float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);

        // Move robot back to original position
        robot.universalMoveRobot(aOpMode, -xVelocity, -yVelocity);
        Thread.sleep(1000);
        robot.stopBaseMotors(aOpMode);

        if ((Math.abs(newMotorPosition - motorPosition) > MECCANUM_WHEEL_ENCODER_MARGIN) ||
                (Math.abs(startingAngle - endingAngle) > 3)) {
            return new TestResult("Platform " + platformName, true);
        } else {
            return new TestResult(platformName, false, "Motors not moving" +
                    " or encoders are faulty");
        }
    }

    private class TestPlatformForward implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericPlatformTest("forwards/backwards", 0.0f, 0.7f);
        }
    }

    private class TestPlatformLeft implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericPlatformTest("sideways", 0.7f, 0.0f);
        }
    }

    private class TestPlatformDiagonal implements AutomaticTest {
        @SuppressWarnings("ConstantConditions")
        public TestResult runTest() throws InterruptedException {
            // We will move the robot in 4 phases to test all 4 diagonals.
            // We will pick representative motors based on the specific diagonal as all
            //wheels will not rotate with power during diagonal moves.
            // We also check for greater than 3 degree rotation as a means to check if there is platform
            //rotation occurring during these moves. If the platform is rotating during these moves, this will
            //lead to inaccuracy and may be symptomatic of other underlying problems such as
            //irregular weight distribution and lack of coplanar 4 wheel contact.

            //phase 1: Move Robot 45 degrees.
            boolean failedDiagTopRight = false;
            boolean failedDiagTopRightRotation = false;

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, 45, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);
            float rotationDiagTopRightAngle = Math.abs(startingAngle - endingAngle);

            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagTopRight = true;
            }
            if (rotationDiagTopRightAngle > 3) {
                failedDiagTopRightRotation = true;
            }

            //phase 2: Move Robot 135 degrees
            boolean failedDiagBottomRight = false;
            boolean failedDiagBottomRightRotation = false;

            motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR); //representative motor
            startingAngle = robot.getBoschGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, 135, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            endingAngle = robot.getBoschGyroSensorHeading(aOpMode);
            float rotationDiagBottomRightAngle = Math.abs(startingAngle - endingAngle);

            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagBottomRight = true;
            }
            if (rotationDiagBottomRightAngle > 3) {
                failedDiagBottomRightRotation = true;
            }

            //phase 3: Move Robot -45 degrees
            boolean failedDiagTopLeft = false;
            boolean failedDiagTopLeftRotation = false;

            motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR); //representative motor
            startingAngle = robot.getBoschGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, -45, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            endingAngle = robot.getBoschGyroSensorHeading(aOpMode);
            float rotationDiagTopLeftAngle = Math.abs(startingAngle - endingAngle);

            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagTopLeft = true;
            }
            if (rotationDiagTopLeftAngle > 3) {
                failedDiagTopLeftRotation = true;
            }


            //phase 4: Move Robot -135 degrees
            boolean failedDiagBottomLeft = false;
            boolean failedDiagBottomLeftRotation = false;

            motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            startingAngle = robot.getBoschGyroSensorHeading(aOpMode); //save starting angle.
            universalMoveRobotPolar(aOpMode, -135, 0.5f, 0.0f, 750, new falseCondition(), false, 0, 0); //move robot diag 45
            newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            endingAngle = robot.getBoschGyroSensorHeading(aOpMode);
            float rotationDiagBottomLeftAngle = Math.abs(startingAngle - endingAngle);


            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                failedDiagBottomLeft = true;
            }

            if (rotationDiagBottomLeftAngle > 3) {
                failedDiagBottomLeftRotation = true;
            }


            // Check if anything went wrong
            if (failedDiagTopLeft || failedDiagTopRight || failedDiagBottomLeft || failedDiagBottomRight) {
                return new TestResult("Platform Diagonal", false, "Motors not moving or " +
                        "encoders are faulty");
            }

            if (failedDiagTopLeftRotation ||
                    failedDiagTopRightRotation ||
                    failedDiagBottomLeftRotation ||
                    failedDiagBottomRightRotation) {
                return new TestResult("Platform Diagonal", false,
                        "Detected too much rotation when performing platform diagonal movements " +
                                "[TDLR:" + rotationDiagTopLeftAngle + "]" +
                                "[TDRR:" + rotationDiagTopRightAngle + "]" +
                                "[BDLR:" + rotationDiagBottomLeftAngle + "]" +
                                "[BDRR:" + rotationDiagBottomRightAngle + "]");
            }

            // Test passed
            return new TestResult("Platform Diagonal", true);
        }

        // Helper method for testPlatformDiagonal
        @SuppressWarnings("SameParameterValue")
        private void universalMoveRobotPolar(rr_OpMode aOpMode, double polarAngle,
                                             double polarVelocity, double rotationalVelocity,
                                             long duration, rr_OpMode.StopCondition condition,
                                             boolean isPulsed, long pulseWidthDuration, long pulseRestDuration)
                throws InterruptedException {
            robot.universalMoveRobotWithCondition(aOpMode, polarVelocity * Math.sin(Math.toRadians(polarAngle)),
                    polarVelocity * Math.cos(Math.toRadians(polarAngle)), rotationalVelocity, duration, condition, isPulsed, pulseWidthDuration, pulseRestDuration);
        }

        // Necessary for testPlatformDiagonal
        private class falseCondition implements rr_OpMode.StopCondition {
            //can be used as an empty condition, so the robot keeps running in universal movement
            public boolean stopCondition(rr_OpMode aOpMode) throws InterruptedException {
                return (false);
            }
        }
    }


    //***************** SERVO TESTS *****************//

    private class TestRelicArm implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setRelicArmPosition(RELIC_ARM_EXTEND_UP);
            Thread.sleep(SERVO_WAIT_TIME);
            robot.setRelicArmGrab();
        }
    }

    private class TestRelicClaw implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setRelicClawOpen();
            Thread.sleep(SERVO_WAIT_TIME);
            robot.setRelicClawClosed();
        }
    }

    private class TestCubePusher implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_RESTED_POSITION);
            Thread.sleep(SERVO_WAIT_TIME);
            robot.setCubePusherPosition(aOpMode, CUBE_PUSHER_INIT_POSITION);
        }
    }

    private class TestJewelArm implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setJewelArmDownPush();
            Thread.sleep(SERVO_WAIT_TIME);
            robot.setJewelPusherNeutral();
        }
    }

    private class TestTrayFlipper implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_SCORING_POSITION);
            Thread.sleep(SERVO_WAIT_TIME);
            robot.setTrayFlipPosition(aOpMode, TRAY_FLIP_COLLECTION_POSITION);
        }
    }


    //***************** CONNECTION TESTS *****************//

    private class TestConnection implements ManualTest {
        public void runTest() throws InterruptedException {
            final int DELAY = 150;
            // Shake the robot around to test connections
            robot.universalMoveRobot(aOpMode, 1, 0);
            Thread.sleep(DELAY);
            robot.universalMoveRobot(aOpMode, -1, 0);
            Thread.sleep(DELAY);
            robot.universalMoveRobot(aOpMode, 1, 1);
            Thread.sleep(DELAY);
            robot.universalMoveRobot(aOpMode, -1, -1);
            Thread.sleep(DELAY);

            robot.turnUsingEncoders(aOpMode, 180, 1, rr_Constants.TurnDirectionEnum.Clockwise);
            Thread.sleep(DELAY);
            robot.turnUsingEncoders(aOpMode, 180, 1, rr_Constants.TurnDirectionEnum.Counterclockwise);
            Thread.sleep(DELAY);
            robot.turnUsingEncoders(aOpMode, 90, 1, rr_Constants.TurnDirectionEnum.Clockwise);
            Thread.sleep(DELAY);
            robot.turnUsingEncoders(aOpMode, 90, 1, rr_Constants.TurnDirectionEnum.Counterclockwise);
            Thread.sleep(DELAY);

            robot.stopBaseMotors(aOpMode);
        }
    }
}
