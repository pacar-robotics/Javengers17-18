package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by accio on 1/14/2018.
 */

@TeleOp(name = "JewelDetectorTest", group = "Autonomous")
public class TestJewelDetector extends rr_OpMode {
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing");
        telemetry.update();

        JewelDetector jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.enable();

        telemetry.addLine("Done Initializing");
        telemetry.update();

        waitForStart();

        telemetry.clear();

        while (opModeIsActive()) {
            telemetry.addLine(String.valueOf(jewelDetector.getCurrentOrder()));
            telemetry.update();
            Thread.sleep(500);
        }
    }
}
