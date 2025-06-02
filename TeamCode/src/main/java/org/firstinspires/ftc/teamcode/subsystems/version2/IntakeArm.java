package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MiscMethods;

import java.util.HashMap;

public class IntakeArm {
    double ARM_SPEED = 0.003;
    double WRIST_SPEED = 0.003;

    //Positions will be set relative to Parallel Position to simplify future repair efforts
    static double ARM_PARALLEL = 0.783; //Left Servo Parallel
    static double WRIST_PARALLEL = 0.691; //Wrist Servo Parallel
    public enum IntakeArmStates{
        reset(0,0),

        TRANSFER(ARM_PARALLEL - 0.485,WRIST_PARALLEL-0.738),
        HOVER(ARM_PARALLEL + 0.08,WRIST_PARALLEL + 0.093),//0.846,0.805
        INTAKE(ARM_PARALLEL + 0.11,WRIST_PARALLEL),
        PUSH(ARM_PARALLEL+0.179, WRIST_PARALLEL+0.195),
        SAMPLE_INTAKE_AUTO(ARM_PARALLEL + 0.17, WRIST_PARALLEL + 0.15);//0.841,0.629

        private double arm;
        private double wrist;
        IntakeArmStates(double armPos, double wristPos) {
            arm = armPos;
            wrist = wristPos;
        }
        double getArm() {
            return arm;
        }
        double getWrist(){
            return wrist;
        }
        void setArm(double armPos){
            arm = armPos;
        }
        void setWrist(double wristPos){
            wrist = wristPos;
        }
        void setArmWrist(double armPos, double wristPos){
            setArm(armPos);
            setWrist(wristPos);
        }

        public void resetPositions(){
            TRANSFER.setArmWrist(ARM_PARALLEL - 0.485,WRIST_PARALLEL-0.738);
            HOVER.setArmWrist(ARM_PARALLEL + 0.08,WRIST_PARALLEL + 0.093);
            INTAKE.setArmWrist(ARM_PARALLEL + 0.16,WRIST_PARALLEL + 0.112); //0.917, 0.803
        }
    }




    Servo armPivotLeft, armPivotRight;
    Servo wristServo;
    HardwareMap hardwareMap;

    double pivRightOffset = -0.0417;

    IntakeArmStates currentState;


    public IntakeArm(HardwareMap hw) {
        this(hw, "pivLeft", "pivRight","wrist");
    }

    public IntakeArm(HardwareMap hw, String pivLeftName, String pivRightName, String wristName){
        armPivotLeft = hw.get(Servo.class,pivLeftName);
        armPivotRight = hw.get(Servo.class,pivRightName);
        wristServo = hw.get(Servo.class,wristName);

        armPivotRight.setDirection(Servo.Direction.REVERSE);



        hardwareMap=hw;
        currentState = IntakeArmStates.TRANSFER;
        IntakeArmStates.reset.resetPositions();
    }

    public void init(){
        goToPosition(IntakeArmStates.TRANSFER);
    }

    public void goToPosition(IntakeArmStates state){
        currentState = state;
        setPositionArm(state.getArm());
        setPositionWrist(state.getWrist());
    }


    /**
     * Sets the exact position for the subsystem
     * @param position Desired position between [0,1]
     */
    public void setPositionWrist(double position) {
        wristServo.setPosition(position);
    }
    /**
     * Sets the exact position for the subsystem
     * @param position Desired position between [0,1]
     */
    public void setPositionArm(double position) {
        position = MiscMethods.clamp(position,0, 1 - pivRightOffset);
        armPivotLeft.setPosition(position);
        armPivotRight.setPosition(position + pivRightOffset);
    }

    /**
     * Changes the position relative to the current position
     * @param power Speed multiplier, typically between [-1,1] (negatives to change direction)
     */
    public void adjustPositionWrist(double power){
        double position = WRIST_SPEED * power + getPositionWrist();
        position = MiscMethods.clamp(position,0, 1);
        currentState.setWrist(position);
        wristServo.setPosition(position);
    }

    /**
     * Changes the position relative to the current position
     * @param power Speed multiplier, typically between [-1,1] (negatives to change direction)
     */
    public void adjustPositionArm(double power){
        double position = ARM_SPEED * power + getPositionArm();
        position = MiscMethods.clamp(position,0, 1 - pivRightOffset);
        currentState.setArm(position);
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
    public IntakeArmStates getCurrentState(){
        return currentState;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Current State: %s\n"+
                "Left Pivot Position: %f\n" +
                "Right Pivot Position: %f\n" +
                "Right Pivot Offset: %f\n" +
                "Wrist Position: %f\n\n"+
                "Arm From Parallel: %f\n"+ "Wrist From Parallel: %f\n",
                currentState,
                armPivotLeft.getPosition(),
                armPivotRight.getPosition(),
                pivRightOffset,
                wristServo.getPosition(),
                (armPivotLeft.getPosition() - ARM_PARALLEL),
                (wristServo.getPosition() - WRIST_PARALLEL));
    }
}
