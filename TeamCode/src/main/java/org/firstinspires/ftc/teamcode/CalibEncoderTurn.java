package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.rr_Constants.TURN_POWER_FACTOR;


//TODO: WILL MAKE THIS CALIBRATION OP
@Autonomous(name = "Calibrate Encoder Turning", group = "Calibration")
public class CalibEncoderTurn extends rr_OpMode {

    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {

        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);

        waitForStart();


        DBG("Turning 90 degrees clockwise");
        rrAutoLib.robot.turnUsingEncoders(this, 90,TURN_POWER_FACTOR, rr_Constants.TurnDirectionEnum.Clockwise );
        Thread.sleep(5000);

        DBG("Turning 90 degrees anti-clockwise");
        rrAutoLib.robot.turnUsingEncoders(this, 90,TURN_POWER_FACTOR, rr_Constants.TurnDirectionEnum.Counterclockwise );
        Thread.sleep(5000);
    }

}