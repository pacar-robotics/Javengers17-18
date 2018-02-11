package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp J2", group = "TeleOp")
public class TeleOpJ2 extends rr_OpMode {
    rr_TeleLib lib;

    @Override
    public void runOpMode() throws InterruptedException {
        lib = new rr_TeleLib(this, this.hardwareMap);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {

            lib.processTeleOpDrive();
            //lib.processRelicSlide();
            //lib.processRelicClaw();
            //lib.processRelicHand();
            lib.processIntake();
            lib.processCubeAlignment();

            lib.processTrayLift(this);
            lib.processBalance();
            lib.processIMUGyroReset();
            lib.printTelemetry();

            Thread.sleep(10);
        }
    }
}
