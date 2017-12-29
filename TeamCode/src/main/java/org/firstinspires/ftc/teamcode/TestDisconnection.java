package org.firstinspires.ftc.teamcode;

/**
 * Created by Kenneth on 12/29/2017.
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Test Disconnection", group = "Test")

public class TestDisconnection extends rr_OpMode{

    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {

        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);

        waitForStart();


        DBG("move forward 10 inches");
        rrAutoLib.robot.moveRobotToPositionFB(this, 10f, .7f, false);
        Thread.sleep(250);
        DBG("move backwards 10 inches");
        rrAutoLib.robot.moveRobotToPositionFB(this, -10f, .7f, false);
        Thread.sleep(250);
        DBG("move forward 10 inches");
        rrAutoLib.robot.moveRobotToPositionFB(this, 10f, .7f, false);
        Thread.sleep(250);
        DBG("move backwards 10 inches");
        rrAutoLib.robot.moveRobotToPositionFB(this, -10f, .7f, false);
        Thread.sleep(250);

        DBG("turning clockwise 90");
        rrAutoLib.robot.turnUsingEncodersWithoutRamped(this, 90, .7f, rr_Constants.TurnDirectionEnum.Clockwise);
        Thread.sleep(250);

        DBG("turning counterclockwise 90");
        rrAutoLib.robot.turnUsingEncodersWithoutRamped(this, 90, .7f, rr_Constants.TurnDirectionEnum.Counterclockwise);
        Thread.sleep(250);

        DBG("turning clockwise 90");
        rrAutoLib.robot.turnUsingEncodersWithoutRamped(this, 90, .7f, rr_Constants.TurnDirectionEnum.Clockwise);
        Thread.sleep(250);

        DBG("turning counterclockwise 90");
        rrAutoLib.robot.turnUsingEncodersWithoutRamped(this, 90, .7f, rr_Constants.TurnDirectionEnum.Counterclockwise);
        Thread.sleep(250);

    }

}
