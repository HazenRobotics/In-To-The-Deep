package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Array;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class TelemetryRecorder {
    private ElapsedTime timer;
    private ArrayList<String> messages;

    private Telemetry telemetry;

    public TelemetryRecorder(Telemetry telemetry){
        timer = new ElapsedTime();
        messages = new ArrayList<>();
        this.telemetry = telemetry;
    }

    public void resetTimer(){
        timer.reset();
    }
    public void clearMessages(){
        messages = new ArrayList<>();
    }

    public void addMessage(String message){
        messages.add(message + " " + (timer.time(TimeUnit.MILLISECONDS)/1000.0) + " Elapsed");
        for (String info: messages) {
            telemetry.addLine(info);
        }
        telemetry.update();
    }

    public InstantAction addIntantMessage(String message){
        return new InstantAction(() -> addMessage(message));
    }
}
