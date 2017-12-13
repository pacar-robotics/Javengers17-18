package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Forward;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.SidewaysLeft;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.SidewaysRight;


@Autonomous(name = "Red Position 2", group = "Autonomous")
public class AutoOpRed2 extends rr_OpMode {

    /* Declare OpMode members. */
    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {

        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);
        waitForStart();
        rrAutoLib.redTwoAutonomousCommonAction(this);
    }
}
