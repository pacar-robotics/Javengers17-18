package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.InputMismatchException;
import java.util.Random;

import static org.firstinspires.ftc.teamcode.rr_Constants.GENERIC_TIMER;


@TeleOp(name = "ActionShakeRobotLight", group = "Action")
public class ActionShakeRobotLight extends rr_OpMode {
    rr_AutoLib lib;

    @Override
    public void runOpMode() throws InterruptedException {
        lib = new rr_AutoLib(this, this.hardwareMap);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);
        //lets move the robot in a brief random pattern.
        //in order to test disconnections
        moveRandomly(10000);

        Thread.sleep(5000); //wait to see if there are disconnections
    }

    public void moveRandomly(int shakeTime) throws InterruptedException{
        reset_timer_array(GENERIC_TIMER);
        int shakeCount=0;
        while(time_elapsed_array(GENERIC_TIMER)<shakeTime){
            lib.moveWheels(this,generateRandomDistance(),
                    0.99f,generateRandomDirection(),false);
            telemetryAddData("Shake count","ShakeCount", "["+ shakeCount++ +"]");
            telemetryAddData("Shake time","ShakeTime in ms ",
                    "["+ time_elapsed_array(GENERIC_TIMER) +"]");
            telemetry.update();
        }

    }

    public rr_Constants.DirectionEnum generateRandomDirection(){

        Random rand = new Random();

        // nextInt is normally exclusive of the top value,
        // so add 1 to make it inclusive

        switch(rand.nextInt(4)){
            case 0: return rr_Constants.DirectionEnum.Forward;
            case 1: return rr_Constants.DirectionEnum.Backward;
            case 2: return rr_Constants.DirectionEnum.SidewaysLeft;
            case 3: return rr_Constants.DirectionEnum.SidewaysRight;
            default: return rr_Constants.DirectionEnum.Forward;
        }
    }


    public int generateRandomDistance(){
        final int upperDistance = 6; //6 inches
        final int lowerDistance = 2; //2 inches
        Random rand = new Random();

        // nextInt is normally exclusive of the top value,
        // so add 1 to make it inclusive

        int randomDistance = rand.nextInt((upperDistance - lowerDistance) + 1) + lowerDistance;

        return randomDistance;

    }
}
