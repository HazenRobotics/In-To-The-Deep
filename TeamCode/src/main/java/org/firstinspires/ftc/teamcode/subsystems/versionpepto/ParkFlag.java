package org.firstinspires.ftc.teamcode.subsystems.versionpepto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ParkFlag {

    private Servo parkFlag;

    private double stowPosition = 0.04;
    private double raisedPosition = 0.46;

    private double waveLimit = stowPosition + 0.15;
    private double waveIncrement = 0.01;
    private long lastWaveTime = 0;

    private boolean wave = false;


    public ParkFlag(HardwareMap hw){
        this(hw, "parkFlag");
    }

    public ParkFlag(HardwareMap hw, String name){
        parkFlag = hw.get(Servo.class, name);
    }

    public void stowFlag(){
        parkFlag.setPosition(stowPosition);
        wave = false;
    }

    public void raiseFlag(){
        parkFlag.setPosition(raisedPosition);
        wave = false;
    }

    public void toggleWave(){
        if (wave){
            stowFlag();
        }
        else{
            turnOnWave();
        }
    }

    public void turnOnWave(){
        wave = true;
    }

    public void wave(){
        if (wave && lastWaveTime + 50 < System.currentTimeMillis()){
            lastWaveTime = System.currentTimeMillis();
            parkFlag.setPosition(parkFlag.getPosition() + waveIncrement);
            if (stowPosition > parkFlag.getPosition() || parkFlag.getPosition() > waveLimit){
                waveIncrement *= -1;
            }
        }
    }

    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Servo Position: %f\n"+
                "Wave Enabled: %b",
                parkFlag.getPosition(),
                wave);
    }
}
