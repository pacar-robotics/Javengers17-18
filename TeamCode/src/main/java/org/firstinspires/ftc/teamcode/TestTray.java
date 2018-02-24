package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_HORIZONTAL_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_FLIP_SCORING_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_COLLECTION_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_HEIGHT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRAY_LIFT_POWER;
import static org.firstinspires.ftc.teamcode.rr_Constants.TRIGGER_THRESHOLD;

@TeleOp(name = "TestTray", group = "Test")
public class TestTray extends rr_OpMode {
    rr_Robot robot;



    public void runOpMode() throws InterruptedException {


        robot = new rr_Robot(this);
        robot.teleopInit(this, this.hardwareMap);

        //set the tray servo to a starting position
        robot.setTrayFlipPosition(this, robot.trayFlipPosition);

        waitForStart();

        telemetry.setAutoClear(true);

        while (opModeIsActive()) {

            telemetry.addLine("Tray Flip Pos: " + robot.trayFlipPosition +
                    "Tray Height Pos:"+ robot.trayHeightPosition);
            telemetry.addLine("Use Left and Right Trigger to raise and lower");
            telemetry.addLine("Use X & B to flip tray, A and Y to rotate tray" );
            processTrayFlipAdjustments();
            processTrayHeightAdjustments();
            telemetry.update();

        }

    }
    private void processTrayHeightAdjustments() throws InterruptedException{

        if(gamepad1.right_trigger>TRIGGER_THRESHOLD){
            //raise the height of the tray
            if(robot.trayHeightPosition>=TRAY_HEIGHT_MAX_POSITION){ //check for limit
                robot.trayHeightPosition=TRAY_HEIGHT_MAX_POSITION;
            }else{
                robot.trayHeightPosition+=50;
            }
            robot.setTrayHeightPositionWithTouchLimits(this, robot.trayHeightPosition, TRAY_LIFT_POWER);
        }
        if(gamepad1.left_trigger>TRIGGER_THRESHOLD){
            //lower the height of the tray
            if(robot.trayHeightPosition<=TRAY_HEIGHT_COLLECTION_POSITION){ //check for limit
                robot.trayHeightPosition=TRAY_HEIGHT_COLLECTION_POSITION;
            }else{
                robot.trayHeightPosition-=50;
            }
            robot.setTrayHeightPositionWithTouchLimits(this, robot.trayHeightPosition, TRAY_LIFT_POWER);
        }





    }
    private void processTrayFlipAdjustments() throws InterruptedException{
            if(gamepad1.y){
                //increase angle of tray flip to be more vertical.
                if(robot.trayFlipPosition>=TRAY_FLIP_SCORING_POSITION){ //check for limit
                    robot.trayFlipPosition=TRAY_FLIP_SCORING_POSITION;
                }else{
                    robot.trayFlipPosition+=0.025;
                }
                robot.setTrayFlipPosition(this, robot.trayFlipPosition);
                Thread.sleep(250);
            }
            if(gamepad1.a){
                //decrease angle of tray flip to be more vertical.
                if(robot.trayFlipPosition<=TRAY_FLIP_COLLECTION_POSITION){ //check for limit
                    robot.trayFlipPosition=TRAY_FLIP_COLLECTION_POSITION;
                }else{
                    robot.trayFlipPosition-=0.025;
                }
            robot.setTrayFlipPosition(this, robot.trayFlipPosition);
                Thread.sleep(250);
        }
        if(gamepad1.x){
            //set tray to collection
            robot.trayFlipPosition=TRAY_FLIP_COLLECTION_POSITION;
            robot.setTrayFlipPosition(this, robot.trayFlipPosition);
            Thread.sleep(250);
        }
        if(gamepad1.b){
            //set tray to scoring
            robot.trayFlipPosition=TRAY_FLIP_SCORING_POSITION;
            robot.setTrayFlipPosition(this, robot.trayFlipPosition);
            Thread.sleep(250);
        }
        if(gamepad1.left_bumper){
            //set tray to horizontal
            robot.trayFlipPosition=TRAY_FLIP_HORIZONTAL_POSITION;
            robot.setTrayFlipPosition(this, robot.trayFlipPosition);
            Thread.sleep(250);
        }

    }

}

