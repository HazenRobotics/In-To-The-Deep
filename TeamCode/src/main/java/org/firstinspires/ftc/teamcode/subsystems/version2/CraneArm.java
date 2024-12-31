package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CraneArm {
    Servo forward,backward;
    private double offset = 0;
    private int face = -1;
    public CraneArm(HardwareMap hw) {
        forward = hw.get(Servo.class,"forward");
        backward = hw.get(Servo.class,"backward");
    }
    public void setPosition(double p) {
        forward.setPosition(p);
        backward.setPosition(face*p+offset);
    }
    public void setOffset(double o) {
        backward.setPosition(backward.getPosition()+o);

    }
    public void swap(){
        face=1;
    }
    public void swap1() {
        face=-1;
    }

}