package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.rr_Constants.DirectionEnum.Backward;
import static org.firstinspires.ftc.teamcode.rr_Constants.FRONT_LEFT_MOTOR;


@TeleOp(name = "Test Wheels Debug", group = "Test")
public class TestWheelsDebug extends rr_OpMode {

    /* Declare OpMode members. */
    rr_AutoLib rrAutoLib;

    @Override
    public void runOpMode() throws InterruptedException {


        //Initialize library which in turn initializes the robot plus its hardware map
        //We need to pass the this pointer into vv_Lib in order to call some value added functions
        //in vv_Opmode

        telemetryAddData("Initializing", ":Please", "wait..");
        telemetryUpdate();
        DBG("before try");

        DBG("Before vvLIb init");
        rrAutoLib = new rr_AutoLib(this, this.hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver", "Im Ready");    //
        telemetry.update();
        DBG("before waitForStart");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetryAddData("Test", "Move",
                "Moving Left Front wheel Forwardinches");
        rrAutoLib.robot.rotateWheel(this, FRONT_LEFT_MOTOR,
                rr_Constants.DirectionEnum.Forward);
        telemetryUpdate();

        Thread.sleep(500);

        telemetryAddData("Test", "Move",
                "Moving Left Rear wheel Forwardinches");
        rrAutoLib.robot.rotateWheel(this, BACK_LEFT_MOTOR,
                rr_Constants.DirectionEnum.Forward);
        telemetryUpdate();

        Thread.sleep(500);

        telemetryAddData("Test", "Move",
                "Moving Right Front wheel Forwardinches");
        rrAutoLib.robot.rotateWheel(this, rr_Constants.FRONT_RIGHT_MOTOR,
                rr_Constants.DirectionEnum.Forward);
        telemetryUpdate();

        Thread.sleep(500);

        telemetryAddData("Test", "Move",
                "Moving Right Back wheel Forwardinches");
        rrAutoLib.robot.rotateWheel(this, BACK_RIGHT_MOTOR,
                rr_Constants.DirectionEnum.Forward);
        telemetryUpdate();

    }

}