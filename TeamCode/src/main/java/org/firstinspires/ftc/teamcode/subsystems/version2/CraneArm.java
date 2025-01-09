package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.MiscMethods;

public class CraneArm {
    double SPEED = 0.001;
    double OPEN = 0.65;
    double OPEN_FULL = 0.5;
    double CLOSE = 0.95;
    double DELAY_TIME = 1000; //Milliseconds

    double LIFT_CROSS = 0.5;
    private double offsetBackward = 0;

    public enum CraneStates{
        TRANSFER(0.723,0, true),
        SPECIMEN_INTAKE(0.1,-0.014, true),
        SPECIMEN_DEPOSIT(0.85,0.1, false),
        SAMPLE_DEPOSIT(0.1,-0.1, false);

        private double position, rotation;
        private boolean isOpen;
        CraneStates(double pos, double rot, boolean open){
            position = pos;
            rotation = rot;
            isOpen = open;
        }
        double getPosition(){
            return position;
        }
        double getRotation(){
            return rotation;
        }
        boolean getIsOpen(){
            return isOpen;
        }

        void setPos(double pos){
            position = pos;
        }
        void setRot(double rot){
            rotation = rot;
        }
    }

    ServoImplEx forward,backward;
            Servo claw;

    private int face = -1;
    boolean isClawOpen;
    boolean servoDelay, clawDelay;
    double clawDelayTime;
    ElapsedTime timerCrane, timerClaw;

    CraneStates currentState;



    public CraneArm(HardwareMap hw) {
        forward = hw.get(ServoImplEx.class,"forward");
        backward = hw.get(ServoImplEx.class,"backward");
        forward.setDirection(Servo.Direction.REVERSE);

        forward.setPwmRange(new PwmControl.PwmRange(500,2500));
        backward.setPwmRange(new PwmControl.PwmRange(500,2500));

        claw = hw.get(Servo.class,"claw");
        currentState = CraneStates.TRANSFER;

        timerCrane = new ElapsedTime();
        timerClaw = new ElapsedTime();
        servoDelay = false;
        clawDelay = false;
    }

    public void init(){
        openClaw();
        goToPosition(CraneStates.TRANSFER);
    }

    public void goToPosition(CraneStates state){
        if (state.getIsOpen() && checkLiftCrossing(currentState, state)){
            openClawFull(500);
        } else{
            openClawFull(0);
        }

        currentState = state;
        if (state != CraneStates.SAMPLE_DEPOSIT) {
            updateServoPosition();
        }else{
            servoDelay = true;
            timerCrane.reset();
            forward.setPosition(currentState.getPosition());
            backward.setPosition(currentState.getPosition() + offsetBackward);
        }
    }


    //Claw Methods
    public void openClaw(){
        claw.setPosition(OPEN);
        isClawOpen = true;
    }
    public void openClawFull(double delayMiliseconds){
        if (delayMiliseconds < 1){
            claw.setPosition(OPEN_FULL);
        }else{
            clawDelay = true;

        }
    }
    public void closeClaw(){
        claw.setPosition(CLOSE);
        isClawOpen = false;
    }

    public void toggleClaw(){
        if (isClawOpen){
            closeClaw();
        }else{
            openClaw();
        }
    }

    public void adjustPositionRotation(double powerPos, double powerRot){
        if(powerPos == 0 && powerRot == 0){
            return;
        }
        currentState.setPos(MiscMethods.clamp(currentState.getPosition() + powerPos * SPEED, 0.1, 0.9));
        currentState.setRot(MiscMethods.clamp(currentState.getRotation() + powerRot * SPEED, -0.1, 0.1));
        updateServoPosition();
    }


    public void updateServoPosition(){
        forward.setPosition(currentState.getPosition() + currentState.getRotation());
        backward.setPosition(currentState.getPosition() - currentState.getRotation() + offsetBackward);
    }

    public void checkDelay(){
        if (servoDelay && timerCrane.milliseconds() > DELAY_TIME){
            updateServoPosition();
            timerCrane.reset();
            servoDelay = false;
        }
        if (clawDelay && timerClaw.milliseconds() > clawDelayTime){
            openClawFull(0);
            timerClaw.reset();
            clawDelay = false;
        }
    }

    /**
     * Checks if the crane will have to cross the lift cross
     * @param curState Current State
     * @param nextState Next State
     * @return If this does cross, return true, otherwise false
     */
    public boolean checkLiftCrossing(CraneStates curState, CraneStates nextState){
        boolean curForward = curState.getPosition() > LIFT_CROSS;
        boolean nextForward = nextState.getPosition() > LIFT_CROSS;
        return curForward ^ nextForward;
    }

//    public void setPosition(double p) {
//        forward.setPosition(p);
//        backward.setPosition(face*p+offset);
//    }
//    public void setOffset(double o) {
//        backward.setPosition(backward.getPosition()+o);
//    }

    public void swap(){
        face=1;
    }
    public void swap1() {
        face=-1;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Crane Linear Pos: %f\n" +
                "Crane Rotational Pos: %f\n" +
                "Backward Offset: %f\n"+
                "Claw isOpen: %b\n",
                currentState.getPosition(),
                currentState.getRotation(),
                forward.getPosition() - backward.getPosition(),
                isClawOpen);
    }

}