package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class TelemetryRecorder {
    private ElapsedTime timer;
    private ArrayList<String> messages;
    private Telemetry telemetry;

    /**
     * Constructor
     * @param telemetry LinearOpMode's telemetry object
     */
    public TelemetryRecorder(Telemetry telemetry){
        timer = new ElapsedTime();
        messages = new ArrayList<>();
        this.telemetry = telemetry;
    }

    /**
     * Resets the current timer
     */
    public void resetTimer(){
        timer.reset();
    }

    /**
     * Clears all messages in telemetry.
     */
    public void clearMessages(){
        messages = new ArrayList<>();
        telemetry.update();
    }

    /**
     * Adds a message to the telemetry stream with an associated time stamp
     * @param message String message to send to telemetry
     */
    public void addMessage(String message){
        messages.add(message + " " + (timer.time(TimeUnit.MILLISECONDS)/1000.0) + " Elapsed");
        for (String info: messages) {
            telemetry.addLine(info);
        }
        telemetry.update();
    }

    /**
     * A method to save a few characters when sending telemetry messages during roadrunner paths
     * @param message String message to send to telemetry
     * @return InstantAction Lambda of addMessage
     */
    public InstantAction addInstantMessage(String message){
        return new InstantAction(() -> addMessage(message));
    }

    /**
     * @return Returns the current time since reset
     */
    public double getTime(){
        return timer.time(TimeUnit.SECONDS);
    }
}
