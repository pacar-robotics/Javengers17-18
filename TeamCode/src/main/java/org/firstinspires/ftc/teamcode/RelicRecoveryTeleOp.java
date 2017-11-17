package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class RelicRecoveryTeleOp extends rr_OpMode {
    rr_TeleLib lib;

    @Override
    public void runOpMode() throws InterruptedException {
        lib = new rr_TeleLib(this, this.hardwareMap);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {
            lib.processClaw();
            lib.processOrientationClaw();
            lib.processCubeArm();
            lib.processFieldOrientedDrive();
            lib.processRelicSlide();
            lib.processRelicClaw();
            lib.processRelicHand();
            lib.printTelemetry();

            Thread.sleep(10);
        }
    }
}
