package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.CUBE_PUSHER_RESTED_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_CARRY;
import static org.firstinspires.ftc.teamcode.rr_Constants.RELIC_ARM_INIT;


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
        //lets move the relic arm up to rest position.
        lib.robot.setRelicArmPosition(RELIC_ARM_CARRY);

        while (opModeIsActive()) {

            lib.processTeleOpDrive();
            lib.processRelicSlide();
            lib.processRelicClaw();
            lib.processRelicArm();
            lib.processRelicDropoff();
            lib.processRelicArmAdjustments();
            lib.processRelicWristAdjustments();
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
