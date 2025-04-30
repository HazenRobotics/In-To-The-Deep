package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PivotPractice {
    Servo pivotServo;
    double position;
    public  PivotPractice(HardwareMap hardwareMap, String pivot){
        pivotServo=hardwareMap.get(Servo.class, pivot);
    }
    public void adjustPivotServo(double increment){

        position +=  increment*0.001;
        pivotServo.setPosition(position);
    }
    public double getPosition() {
        return pivotServo.getPosition();

    }
}
