package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ActionSetCubeArmToIntake", group = "Action")
public class ActionSetCubeArmToIntake extends rr_OpMode {
    rr_Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap); //init lowers the arm to intake
    }

}

