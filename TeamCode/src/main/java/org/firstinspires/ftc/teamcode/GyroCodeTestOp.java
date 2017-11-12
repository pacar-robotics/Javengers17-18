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


        DBG("turning clockwise 90");
        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,90);
        Thread.sleep(250);
        DBG("turning clockwise to 180 total, 90 from last");
        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,180);
        Thread.sleep(250);
        DBG("zeroing the Gyro, by re-initialization");
        rrAutoLib.robot.setBoschGyroZeroYaw(this);
        DBG("If zeroing worked, this should cause turn clockwise 90 from last");
        rrAutoLib.robot.turnAbsoluteBoschGyroDegrees(this,90);
        Thread.sleep(300000);

    }

}