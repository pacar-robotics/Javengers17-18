package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class rr_DiagLib {

    rr_Robot robot;
    rr_OpMode aOpMode;

    public rr_DiagLib(rr_OpMode aOpMode, HardwareMap aHwMap) throws InterruptedException {
        robot = new rr_Robot(aOpMode, aHwMap);
        this.aOpMode = aOpMode;
    }
}
