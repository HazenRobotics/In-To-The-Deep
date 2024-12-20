package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: Conner Implements Deposit Slide
public class DepositSlide {

    Servo slideLeft, slideRight;
    HardwareMap hw;
    public DepositSlide(HardwareMap hw) {
        slideLeft = hw.get(Servo.class,"slideleft");
        slideRight = hw.get(Servo.class,"slideright");
        this.hw=hw;

    }
    public void setPosition(double position) {
        slideLeft.setPosition(position);
        slideRight.setPosition(position);
    }
    public void sendIn() {
        setPosition(0);
    }
    public void sendOut() {
        setPosition(0.5);
    }


}
