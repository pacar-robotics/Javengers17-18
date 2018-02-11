package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;

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
        //lets move the cube pusher arm to the raised state and out of the way.
        lib.robot.setCubePusherPosition(this, CUBE_PUSHER_RESTED_POSITION);

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
