package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: Conner Implements Intake Arm
public class IntakeArm {

    Servo armPivotLeft, armPivotRight;
    Servo wristServo;
    HardwareMap hardwareMap;
    public IntakeArm(HardwareMap hw) {
        armPivotLeft = hw.get(Servo.class,"pivleft");
        armPivotRight = hw.get(Servo.class,"pivright");
        wristServo = hw.get(Servo.class,"wrist");
        hardwareMap=hw;
    }
    public void setPositionWrist(double position) {
        wristServo.setPosition(position);
    }
    public void setPositionArm(double position) {
        armPivotLeft.setPosition(position);
        armPivotRight.setPosition(position);
    }
    public void dropDownArm() {
        setPositionArm(0);
    }
    public void bringUpArm() {
        setPositionArm(0.5);
    }
    public void wristDeposit() {
        setPositionWrist(0.25);
    }
    public void wristDown() {
        setPositionWrist(0);
    }
}
