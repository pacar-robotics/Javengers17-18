package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Random;

import static org.firstinspires.ftc.teamcode.rr_Constants.GENERIC_TIMER;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_HIGH;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_LOW;
import static org.firstinspires.ftc.teamcode.rr_Constants.INTAKE_POWER_MEDIUM;


@Autonomous(name = "ActionRunIntake", group = "Action")
public class ActionRunIntake extends rr_OpMode {
    rr_AutoLib lib;

    @Override
    public void runOpMode() throws InterruptedException {
        lib = new rr_AutoLib(this, this.hardwareMap);

        telemetry.addLine("Ready");
        telemetryUpdate();

        waitForStart();

        telemetry.setAutoClear(true);
        waitForStart();
        runIntake(this);


        Thread.sleep(5000); //wait to see if there are disconnections
    }

 public void runIntake(rr_OpMode aOpMode) throws InterruptedException{
        //start the intake motors.

     while(opModeIsActive()){
         if((lib.robot.getIntakeUltrasonicRightSensorRange(aOpMode)<5)
             ||lib.robot.getIntakeOpticalRightSensorRange(aOpMode)<2){
             //the cube is going in sideways
             //we should switch to low power but reverse one of the motors
             //this should cause the motors to rotate the cube so it is straight
             //this has to be tested and adjusted.
             lib.robot.setIntakePower(aOpMode, INTAKE_POWER_MEDIUM, -INTAKE_POWER_MEDIUM);
             Thread.sleep(200); //wait for a second for rotation.
             lib.robot.setIntakePower(aOpMode, 0, 0);
             Thread.sleep(100); //wait for a second for rotation.
         }else{
             lib.robot.setIntakePower(aOpMode, INTAKE_POWER_LOW, INTAKE_POWER_LOW);
         }

     }
 }

}
