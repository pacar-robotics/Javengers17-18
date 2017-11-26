package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test Joystick Driving", group = "Test")

public class TestDriveSystemTele extends rr_OpMode {
    rr_Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new rr_Robot(this, this.hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.universalMoveRobot(this, gamepad1.left_stick_x, gamepad1.left_stick_y);
        }
    }
}
