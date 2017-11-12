package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoOpBlueOne", group = "Autonomous")
public class AutoOpBlueOne extends rr_OpMode {

    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {

        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);

        waitForStart();
        rrAutoLib.blueOneAutonomousCommonAction(this);
        Thread.sleep(6000);

    }

}