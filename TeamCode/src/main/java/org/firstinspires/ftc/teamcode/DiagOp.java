package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp(name = "Diagnostics", group = "Diag")
public class DiagOp extends rr_OpMode {

    private rr_DiagLib diagLib;
    private ArrayList<rr_DiagLib.TestResult> testResults;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();


    }

    private void initialize() throws InterruptedException {
        diagLib = new rr_DiagLib(this, this.hardwareMap);
        testResults = new ArrayList<>();
        telemetry.setAutoClear(false);
    }

    private void runAutomaticTests() throws InterruptedException {
        for (rr_DiagLib.RobotTest robotTest : diagLib.robotTests) {
            if (robotTest.getTestType() == rr_DiagLib.TestType.AUTOMATIC) {
                runTest(robotTest);
            }
        }
    }

    private void runAllTests() throws InterruptedException {
        for (rr_DiagLib.RobotTest robotTest : diagLib.robotTests) {
            runTest(robotTest);
        }
    }

    private void runTest(rr_DiagLib.RobotTest robotTest) throws InterruptedException {
        telemetry.clear();
        telemetry.addLine("Running: " + robotTest.getTestName());
        telemetry.update();

        // Runs and stores test result
        rr_DiagLib.TestResult testResult = robotTest.getTestMethod().runTest();

        telemetry.addLine("Test " + (testResult.getTestResult() ? "PASSED" : "FAILED"));
        telemetry.addLine(testResult.getTestMessage());
        telemetry.update();

        // Adds test result to list to be reviewed at the end
        testResults.add(testResult);
    }
}
