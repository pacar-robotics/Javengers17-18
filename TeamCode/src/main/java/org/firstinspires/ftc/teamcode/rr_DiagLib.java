package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_ARM;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_WINCH;

class rr_DiagLib {

    // Used for motor and platform movements
    private final static float MECCANUM_WHEEL_ENCODER_MARGIN = 50;

    private rr_Robot robot;
    private rr_OpMode aOpMode;
    ArrayList<RobotTest> robotTests;

    rr_DiagLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode);
        robot.autonomousInit(aOpMode, aHwMap);
        this.aOpMode = aOpMode;

        robotTests = new ArrayList<>();
        initializeRobotTests();
    }


    //***************** INNER CLASSES *****************//

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

    enum TestType {AUTOMATIC, MANUAL}

    interface AutomaticTest {
        TestResult runTest() throws InterruptedException;
    }

    interface ManualTest {
        void runTest() throws InterruptedException;
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

        public void setTestMessage(String testMessage) {
            this.testMessage = testMessage;
        }

        public String getElementName() {
            return elementName;
        }

        public boolean getTestResult() {
            return testResult;
        }

        public String getTestMessage() {
            return testMessage;
        }
    }


    //***************** CLASS METHODS *****************//

    private void initializeRobotTests() {
        robotTests.add(new RobotTest("Front Left Wheel", TestType.AUTOMATIC, new TestFrontLeftWheel()));
        robotTests.add(new RobotTest("Front Right Wheel", TestType.AUTOMATIC, new TestFrontRightWheel()));
        robotTests.add(new RobotTest("Back Left Wheel", TestType.AUTOMATIC, new TestBackLeftWheel()));
        robotTests.add(new RobotTest("Back Right Wheel", TestType.AUTOMATIC, new TestBackRightWheel()));
        robotTests.add(new RobotTest("Cube Arm Motor", TestType.AUTOMATIC, new TestCubeArmMotor()));
        robotTests.add(new RobotTest("Relic Winch Motor", TestType.AUTOMATIC, new TestRelicWinchMotor()));

        robotTests.add(new RobotTest("Platform Forward", TestType.AUTOMATIC, new TestPlatformForward()));
        robotTests.add(new RobotTest("Platform Left", TestType.AUTOMATIC, new TestPlatformLeft()));
        robotTests.add(new RobotTest("Platform Diagonal", TestType.AUTOMATIC, new TestPlatformDiagonal()));


        robotTests.add(new RobotTest("Cube Claw", TestType.MANUAL, new TestCubeClaw()));
        robotTests.add(new RobotTest("Relic Arm", TestType.MANUAL, new TestRelicArm()));
        robotTests.add(new RobotTest("Relic Claw", TestType.MANUAL, new TestRelicClaw()));

        robotTests.add(new RobotTest("Jewel Arm", TestType.MANUAL, new TestJewelArm()));
        robotTests.add(new RobotTest("Jewel Pusher", TestType.MANUAL, new TestJewelPusher()));
    }


    //***************** MOTOR TESTS *****************//

    private TestResult genericMotorTest(int motor, String motorName, boolean defaultDirection,
                                        boolean resetMotor) throws InterruptedException {
        robot.setMotorMode(aOpMode, motor, DcMotor.RunMode.RUN_USING_ENCODER);
        int motorPosition = robot.getMotorPosition(aOpMode, motor);
        // Runs motor for 1 second at half speed
        robot.testMotor(aOpMode, motor, 0.5f * (defaultDirection ? 1 : -1), 1000);
        int newMotorPosition = robot.getMotorPosition(aOpMode, motor);

        // Move motor back if requested
        if (resetMotor) {
            robot.testMotor(aOpMode, motor, -0.5f * (defaultDirection ? 1 : -1), 1000);
        }

        // If the positions are different enough, the motor is running and working
        if (Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) {
            return new TestResult(motorName, true);
        } else {
            return new TestResult(motorName, false, "Failed to detect rotation");
        }
    }

    private class TestFrontLeftWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(FRONT_LEFT_MOTOR, "Front left wheel", true, false);
        }
    }

    private class TestFrontRightWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(FRONT_RIGHT_MOTOR, "Front right wheel", true, false);
        }
    }

    private class TestBackLeftWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(BACK_LEFT_MOTOR, "Back left wheel", true, false);
        }
    }

    private class TestBackRightWheel implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(BACK_RIGHT_MOTOR, "Back right wheel", true, false);
        }
    }

    private class TestCubeArmMotor implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(CUBE_ARM, "Cube arm motor", false, true);
        }
    }

    private class TestRelicWinchMotor implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            return genericMotorTest(RELIC_WINCH, "Relic winch motor", true, true);
        }
    }


    //***************** SERVO TESTS *****************//

    private class TestCubeClaw implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.closeCubeClawServoTwoCube();
            robot.closeCubeClawServoOneCube();
        }
    }

    private class TestRelicArm implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setRelicArmExtend();
            robot.setRelicArmGrab();
        }
    }

    private class TestRelicClaw implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setRelicClawOpen();
            robot.setRelicClawClosed();
        }
    }

    private class TestJewelArm implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.setJewelArmDownPush();
            robot.setJewelArmUp();
        }
    }

    private class TestJewelPusher implements ManualTest {
        public void runTest() throws InterruptedException {
            robot.pushRightJewel();
            robot.pushLeftJewel();
            robot.setJewelPusherNeutral();
        }
    }


    //***************** PLATFORM TESTS *****************//

    private class TestPlatformForward implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); // Representative motor
            float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); // Save starting angle.
            robot.universalMoveRobot(aOpMode, 0.5f, 0.0f);
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);

            if ((Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
                return new TestResult("Platform forwards/backwards", true);
            } else {
                return new TestResult("Platform forwards/backwards", false, "Failed to detect platform movement");
            }
        }
    }

    private class TestPlatformLeft implements AutomaticTest {
        public TestResult runTest() throws InterruptedException {
            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); // Representative motor
            float startingAngle = robot.getBoschGyroSensorHeading(aOpMode); // Save starting angle.
            robot.universalMoveRobot(aOpMode, 0.0f, 0.5f);
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getBoschGyroSensorHeading(aOpMode);

            if ((Math.abs(newMotorPosition - motorPosition) < MECCANUM_WHEEL_ENCODER_MARGIN) || (Math.abs(startingAngle - endingAngle) > 3)) {
                return new TestResult("Platform sideways", true);
            } else {
                return new TestResult("Platform sideways", true, "Failed to detect platform movement");
            }
        }
    }

    private class TestPlatformDiagonal implements AutomaticTest {
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
                return new TestResult("Platform Diagonal", false, "Failed to detect platform diagonal movement");
            }

            if (failedDiagTopLeftRotation ||
                    failedDiagTopRightRotation ||
                    failedDiagBottomLeftRotation ||
                    failedDiagBottomRightRotation) {
                return new TestResult("Platform Diagonal", false,
                        "Detected too much rotation when performing platform diagonal movements" +
                                "[TDLR:" + rotationDiagTopLeftAngle + "]" +
                                "[TDRR:" + rotationDiagTopRightAngle + "]" +
                                "[BDLR:" + rotationDiagBottomLeftAngle + "]" +
                                "[BDRR:" + rotationDiagBottomRightAngle + "]");
            }

            // Test passed
            return new TestResult("Platform Diagonal", true);
        }

        // Helper method for testPlatformDiagonal
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
}
