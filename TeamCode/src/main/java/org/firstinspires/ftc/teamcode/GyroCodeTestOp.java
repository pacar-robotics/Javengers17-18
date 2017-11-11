package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Gyro Code Test Op", group = "Autonomous")
public class GyroCodeTestOp extends rr_OpMode {

    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {

        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);

        waitForStart();


        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,90);
        Thread.sleep(250);
        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,180);
        Thread.sleep(250);
        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,90);
        Thread.sleep(250);
        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,-180);
        Thread.sleep(300000);

    }

}