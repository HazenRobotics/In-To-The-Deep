package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class IntakeArm {

    public enum IntakeArmStates{
        TRANSFER
    }

    public HashMap<IntakeArmStates,Double> armPositions;
    public HashMap<IntakeArmStates,Double> wristPosition;

    //Positions will be set relative to Parallel Position to simplify future repair efforts
    double ARM_PARALLEL = 0.5; //Left Servo Parallel
    double WRIST_PARALLEL = 0.5; //Wrist Servo Parallel


    Servo armPivotLeft, armPivotRight;
    Servo wristServo;
    HardwareMap hardwareMap;

    double pivRightOffset = -0.0417;


    public IntakeArm(HardwareMap hw) {
        this(hw, "pivLeft", "pivRight","wrist");
    }

    public IntakeArm(HardwareMap hw, String pivLeftName, String pivRightName, String wristName){
        armPivotLeft = hw.get(Servo.class,pivLeftName);
        armPivotRight = hw.get(Servo.class,pivRightName);
        wristServo = hw.get(Servo.class,wristName);

        armPivotRight.setDirection(Servo.Direction.REVERSE);

        hardwareMap=hw;
    }

    public void init(){
        setPositionArm(0.5);
        setPositionWrist(0.5);
    }

    public void initializePositions(){
        armPositions = new HashMap<>();
        wristPosition = new HashMap<>();

        armPositions.put(IntakeArmStates.TRANSFER, 0.304);
        wristPosition.put(IntakeArmStates.TRANSFER, 0.0);
    }
    public void setPositionWrist(double position) {
        wristServo.setPosition(position);
    }
    public void setPositionArm(double position) {
        armPivotLeft.setPosition(position);
        armPivotRight.setPosition(position + pivRightOffset);
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


    public double getPositionArm(){
        return armPivotLeft.getPosition();
    }
    public double getPositionWrist(){
        return wristServo.getPosition();
    }
    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Left Pivot Position: %f\n" +
                "Right Pivot Position: %f\n" +
                "Right Pivot Offset: %f\n" +
                "Wrist Position: %f\n",
                armPivotLeft.getPosition(),
                armPivotRight.getPosition(),
                pivRightOffset,
                wristServo.getPosition());
    }
}
