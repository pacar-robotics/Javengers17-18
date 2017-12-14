package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Main", group = "TeleOp")
public class TeleOpMaryland extends rr_OpMode {
    rr_TeleLib lib;

    @Override
    public void runOpMode() throws InterruptedException {
        lib = new rr_TeleLib(this, this.hardwareMap);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);

        lib.initializeCubeArm(this);

        while (opModeIsActive()) {
            lib.processClaw();
            lib.processOrientationClaw();
            lib.processCubeArm();
            lib.processTeleOpDrive();
        //    lib.processRelicSlide();
            lib.processRelicClaw();
            lib.processRelicHand();
            lib.processBalance();
            lib.processIMUGyroReset();
            lib.printTelemetry();

            Thread.sleep(10);
        }
    }
}
