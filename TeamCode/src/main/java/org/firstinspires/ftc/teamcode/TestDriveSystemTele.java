package org.firstinspires.ftc.teamcode;

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
