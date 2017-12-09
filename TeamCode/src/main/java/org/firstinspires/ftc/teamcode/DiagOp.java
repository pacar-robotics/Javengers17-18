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
    }

    private void runAutomaticTests() throws InterruptedException {
        for (rr_DiagLib.RobotTest robotTest : diagLib.robotTests) {
            if (robotTest.getTestType() == rr_DiagLib.TestType.AUTOMATIC) {
                testResults.add(robotTest.getTestMethod().runTest());
            }
        }
    }

    private void runAllTests() throws InterruptedException {
        for (rr_DiagLib.RobotTest robotTest : diagLib.robotTests) {
            testResults.add(robotTest.getTestMethod().runTest());
        }
    }
}
