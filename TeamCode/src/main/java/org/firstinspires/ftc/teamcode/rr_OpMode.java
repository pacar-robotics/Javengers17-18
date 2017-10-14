package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public abstract class rr_OpMode extends LinearOpMode {

    //This array contains all timers we plan on using throughout our classes
    public static long timer_array[] = new long[5];

    /*
     * If we set the constant DEBUG to true while debugging, all telemetry relating to Debugging
     * will show on the Driver Station. If DEBUG is false, the telemetry will not show up.
     */
    public void DBG(String message) throws InterruptedException {
        if (rr_Constants.DEBUG) {
            telemetry.setAutoClear(rr_Constants.DEBUG_AUTO_CLEAR);
            telemetryAddData("DBG", "Message", ":" + message);
            telemetryUpdate();
            Thread.sleep(rr_Constants.DEBUG_MESSAGE_DISPLAY_TIME);
        }
    }

    public void telemetryAddData(String caption, String key, String message){
        telemetry.addLine(caption).addData(key,message);
    }

    public void telemetryUpdate() {
        telemetry.update();
    }

    public void telemetryAddFormattedData(String caption, String key, int number) {
        telemetry.addLine(caption).addData(key, number);
    }

    public void reset_timer_array(int index) {
        timer_array[index] = System.currentTimeMillis();
    }

    public long time_elapsed_array(int index) {
        //return the time elapsed in milliseconds
        return System.currentTimeMillis() - timer_array[index];
    }

    public interface StopCondition {
        boolean stopCondition(rr_OpMode aOpMode) throws InterruptedException;
    }


}