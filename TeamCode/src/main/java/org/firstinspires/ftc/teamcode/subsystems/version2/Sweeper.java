package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sweeper {

    Servo sweeper;

    double STOWED = 0;
    double OUT = 0.37;

    boolean isStowed;

    public Sweeper(HardwareMap hw){
        this(hw, "sweeper");
    }

    public Sweeper(HardwareMap hw, String name){
        sweeper = hw.get(Servo.class, name);
        isStowed = true;
    }

    public void init(){
        stowSweeper();
    }

    public void stowSweeper(){
        sweeper.setPosition(STOWED);
        isStowed = true;
    }

    public void sweepSweeper(){
        sweeper.setPosition(OUT);
        isStowed = false;
    }

    public void toggleSweeper(){
        if (isStowed){
            sweepSweeper();
        }else{
            stowSweeper();
        }
    }

}
