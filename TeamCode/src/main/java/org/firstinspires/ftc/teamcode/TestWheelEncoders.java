package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;

@Autonomous(name = "Test Wheel Encoders", group = "Calibrations")
public class TestWheelEncoders extends rr_OpMode {

    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {

        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);

        waitForStart();


        DBG("Moving forward 6 inches");
        rrAutoLib.moveWheels(this, 6, 0.5f, rr_Constants.DirectionEnum.Forward, true);
        Thread.sleep(5000);

        DBG("Moving Backward 6 inches");
        rrAutoLib.moveWheels(this, 6, 0.5f, rr_Constants.DirectionEnum.Backward, true);
        Thread.sleep(5000);



    }

}