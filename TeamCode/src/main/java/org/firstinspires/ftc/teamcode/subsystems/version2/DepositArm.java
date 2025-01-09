package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MiscMethods;

public class DepositArm {

    double ARM_SPEED = 0.003;
    double WRIST_SPEED = 0.003;
    static double ARM_PARALLEL = 0.5; //Left Servo Parallel
    static double WRIST_PARALLEL = 0.665; //Wrist Servo Parallel

    double OPEN = 0.5;
    double CLOSE = 1;

    public enum PivotArmStates{
        reset(0,0),
        TRASNFER(ARM_PARALLEL - 0.201, WRIST_PARALLEL - 0.554),//(0.299,0.121),//(ARM_PARALLEL - 0.16, WRIST_PARALLEL - 0.54), //0.34, 0.12
        SPECIMEN_INTAKE(ARM_PARALLEL + 0.31, WRIST_PARALLEL + 0.319),//(0.81,0.984),//(ARM_PARALLEL + 0.293, WRIST_PARALLEL + 0.31), //0.793, 0.97
        SPECIMEN_DEPOSIT(ARM_PARALLEL - 0.119, WRIST_PARALLEL - 0.173),//(0.381,0.492),//(ARM_PARALLEL, WRIST_PARALLEL + 0.19), //0.5, 0.85
        SAMPLE_DEPOSIT(ARM_PARALLEL + 0.041, WRIST_PARALLEL + 0.098);//(0.541,0.763);//(ARM_PARALLEL - 0.09, WRIST_PARALLEL - 0.18);//0.41, 0.48


        private double arm;
        private double wrist;
        PivotArmStates(double armPos, double wristPos) {
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
            TRASNFER.setArmWrist(ARM_PARALLEL - 0.228, WRIST_PARALLEL - 0.534);//0.272, 0.131
            SPECIMEN_INTAKE.setArmWrist(ARM_PARALLEL + 0.298, WRIST_PARALLEL + 0.263);//0.798,0.928
            SPECIMEN_DEPOSIT.setArmWrist(ARM_PARALLEL - 0.149, WRIST_PARALLEL - 0.154);//0.351, 0.511
            SAMPLE_DEPOSIT.setArmWrist(ARM_PARALLEL + 0.041, WRIST_PARALLEL + 0.098);
        }
    }
    Servo armPivotLeft, armPivotRight;
    Servo wristServo;
    Servo claw;

    boolean isClawOpen;

    double pivRightOffset = 0.0;

    PivotArmStates currentState;

    public DepositArm(HardwareMap hw){
        this(hw, "depositForward", "depositBackward", "depositWrist","claw");
    }
    public DepositArm(HardwareMap hw, String pivLeft, String pivRight, String wrist, String clawName){
        armPivotLeft = hw.get(Servo.class,pivLeft);
        armPivotRight = hw.get(Servo.class,pivRight);
        wristServo = hw.get(Servo.class,wrist);

//        armPivotRight.setDirection(Servo.Direction.REVERSE);

        claw = hw.get(Servo.class, clawName);
        PivotArmStates.reset.resetPositions();
    }

    public void init(){
        goToPosition(PivotArmStates.TRASNFER);
        openClaw();
    }

    public void goToPosition(PivotArmStates state){
        currentState = state;
        setPositionArm(state.getArm());
        setPositionWrist(state.getWrist());
    }

    public void setPositionWrist(double position) {
        wristServo.setPosition(position);
    }

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

    public double getPositionArm(){
        return armPivotLeft.getPosition();
    }
    public double getPositionWrist(){
        return wristServo.getPosition();
    }

    public PivotArmStates getCurrentState(){
        return currentState;
    }

    public void openClaw(){
        claw.setPosition(OPEN);
        isClawOpen = true;
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

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Current State: %s\n"+
                        "Left Pivot Position: %f\n" +
                        "Right Pivot Position: %f\n" +
                        "Right Pivot Offset: %f\n" +
                        "Wrist Position: %f\n" +
                        "Claw Open: %b\n",
                currentState,
                armPivotLeft.getPosition(),
                armPivotRight.getPosition(),
                pivRightOffset,
                wristServo.getPosition(),
                isClawOpen);
    }

}
