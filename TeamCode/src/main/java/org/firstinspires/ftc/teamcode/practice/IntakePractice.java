package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakePractice {
    CRServo intakeServo;
    public IntakePractice(HardwareMap hardwareMap, String takeinservo){
        intakeServo=hardwareMap.get(CRServo.class, takeinservo);
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setIntakeServo(double grabby){
        intakeServo.setPower(grabby);
    }
    public void powerOn(){
        intakeServo.setPower(0.7);
    }
    public void powerReverse(){
        intakeServo.setPower(-0.7);
    }
    public void powerOff(){
        intakeServo.setPower(0);
    }
}