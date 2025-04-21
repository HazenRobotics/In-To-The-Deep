package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmPractice {
    CRServo leftservo, rightservo;

    public ArmPractice(HardwareMap hardwareMap,  String leftservo, String rightservo){
        this.leftservo=hardwareMap.get(CRServo.class, leftservo);
        this.rightservo=hardwareMap.get(CRServo.class, rightservo);
        this.rightservo.setDirection(DcMotorSimple.Direction.REVERSE);

    }
     public void moveArm(double duck){
        this.leftservo.setPower(duck);
        this.rightservo.setPower(duck);
     }

     public void setBack(){
        this.leftservo.setPower(0);
        this.rightservo.setPower(0);
     }

}
