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

        runAllTests();

        printResults();

        // Let user read results before program ends
        while (opModeIsActive()) idle();
    }

    private void initialize() throws InterruptedException {
        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing...");
        telemetry.update();

        diagLib = new rr_DiagLib(this, this.hardwareMap);
        testResults = new ArrayList<>();

        telemetry.addLine("Done with Initialization");
        telemetry.update();
    }

    private void runAutomaticTests() throws InterruptedException {
        for (rr_DiagLib.RobotTest robotTest : diagLib.robotTests) {
            if (robotTest.getTestType() == rr_DiagLib.TestType.AUTOMATIC) {
                telemetry.clear();
                telemetry.addLine("Running: " + robotTest.getTestName());
                telemetry.update();

                // Runs and stores test result
                rr_DiagLib.TestResult testResult = robotTest.getAutomaticTest().runTest();

                telemetry.addLine("Test " + (testResult.getTestResult() ? "PASSED" : "FAILED"));
                if (!testResult.getTestResult()) telemetry.addLine(testResult.getTestMessage());
                telemetry.update();

                // Adds test result to list to be reviewed at the end
                testResults.add(testResult);
            }
        }
    }

    private void runAllTests() throws InterruptedException {
        for (rr_DiagLib.RobotTest robotTest : diagLib.robotTests) {
            telemetry.clear();
            telemetry.addLine("Running: " + robotTest.getTestName());
            telemetry.update();

            if (robotTest.getTestType() == rr_DiagLib.TestType.AUTOMATIC) {
                rr_DiagLib.TestResult testResult = robotTest.getAutomaticTest().runTest();

                telemetry.addLine("Test " + (testResult.getTestResult() ? "PASSED" : "FAILED"));
                if (testResult.getTestResult()) telemetry.addLine(testResult.getTestMessage());
                telemetry.update();

                // Adds test result to list to be reviewed at the end
                testResults.add(testResult);
            } else {
                // Manual tests don't have a result
                robotTest.getManualTest().runTest();
            }
        }
    }

    private void printResults() throws InterruptedException {
        telemetry.clearAll();

        for (rr_DiagLib.TestResult result : testResults) {
            if (!result.getTestResult()) {  // Print message if test failed
                telemetry.addLine(result.getElementName() + ":");
                telemetry.addLine("   " + result.getTestMessage());
            }
        }
        telemetry.update();
    }
}
