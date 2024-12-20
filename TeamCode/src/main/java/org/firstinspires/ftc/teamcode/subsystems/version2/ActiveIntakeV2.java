package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: Conner Implements ActiveIntakeV2
public class ActiveIntakeV2 {
    DcMotorEx intake;
    Servo backDoor;
    boolean downDoor=true;
    HardwareMap hardwareMap;

    public ActiveIntakeV2(HardwareMap hw) {
        hardwareMap=hw;
        intake = hw.get(DcMotorEx.class,"intake");
        backDoor = hw.get(Servo.class,"door");
    }
    public void toggleBackDoor() {
        downDoor=!downDoor;
        if(downDoor) {
            backDoor.setPosition(0);
        } else {
            backDoor.setPosition(0.5);
        }
    }
    public void dropDown() {
        backDoor.setPosition(0);
        downDoor=true;
    }
    public void bringU() {
        backDoor.setPosition(0.5);
        downDoor=false;
    }
    public void setPower(double power) {
        intake.setPower(power);
        if(intake.getPower()!=0) {
            dropDown();
        }
    }


}

