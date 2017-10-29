package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Krittika on 10/7/2017.
 */
@TeleOp(name="TeleOpTest", group="TeleOp")

public class TeleOpTest extends LinearOpMode {

    DcMotor frontLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backLeftWheel;
    DcMotor backRightWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while(opModeIsActive()) {

            universalMoveRobotForTeleOp(gamepad1.left_stick_x, gamepad1.left_stick_y);
        }

    }

    private void initialize() {

        frontLeftWheel = hardwareMap.dcMotor.get("motor_front_left");
        frontRightWheel = hardwareMap.dcMotor.get("motor_front_right");
        backLeftWheel = hardwareMap.dcMotor.get("motor_back_left");
        backRightWheel = hardwareMap.dcMotor.get("motor_back_right");

        //setting the direction of each motor
        frontLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public void universalMoveRobotForTeleOp(double xAxisVelocity,
                                            double yAxisVelocity)
            throws InterruptedException {
        double fl_velocity = 0;
        double fr_velocity = 0;
        double bl_velocity = 0;
        double br_velocity = 0;
        double prevFLVelocity = 0;
        double prevFRVelocity = 0;
        double prevBRVelocity = 0;
        double prevBLVelocity = 0;

        //calculate velocities at each wheel.

        //blend with prev velocities to smooth out start

        fl_velocity = ((yAxisVelocity + xAxisVelocity) + prevFLVelocity) / 2;

        fr_velocity = ((yAxisVelocity - xAxisVelocity) + prevFRVelocity) / 2;

        bl_velocity = ((yAxisVelocity - xAxisVelocity) + prevBLVelocity) / 2;

        br_velocity = ((yAxisVelocity + xAxisVelocity) + prevBRVelocity) / 2;

        //save these in variables that are part of vvRobot to be used in next cycle.

        prevFLVelocity = fl_velocity;
        prevFRVelocity = fr_velocity;
        prevBLVelocity = bl_velocity;
        prevBRVelocity = br_velocity;


        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(50);

        //apply specific powers to motors to get desired movement
        //wait till duration is complete.


        frontLeftWheel.setPower(fl_velocity );
        frontRightWheel.setPower(fr_velocity);
        backLeftWheel.setPower(bl_velocity);
        backRightWheel.setPower(br_velocity);

        telemetry.addData("FL:", fl_velocity);
        telemetry.addData("FR:", fr_velocity);
        telemetry.addData("BL:", bl_velocity);
        telemetry.addData("BR:", br_velocity);

        telemetry.addData("FLP:", frontLeftWheel.getPower());
        telemetry.addData("FRP:", frontRightWheel.getPower());
        telemetry.addData("BLP:", backLeftWheel.getPower());
        telemetry.addData("BRP:", backRightWheel.getPower());

        telemetry.update();
    }




}

//END OF PROGRAM
