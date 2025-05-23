package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MiscMethods;

public class DepositArmV2 {
    public enum PivotArmStates{
        reset(0,0),
        TRASNFER(ARM_PARALLEL - 0.189, WRIST_PARALLEL - 0.76),
        SPECIMEN_INTAKE(ARM_PARALLEL + 0.133, WRIST_PARALLEL + 0.36),
        SPECIMEN_DEPOSIT(ARM_PARALLEL - 0.329, WRIST_PARALLEL + 0.219),
        SAMPLE_DEPOSIT(ARM_PARALLEL + 0.151, WRIST_PARALLEL - 0.016),

        TEMP_TRANSFER(ARM_PARALLEL, WRIST_PARALLEL - 0.76),
        PARK_ARM(ARM_PARALLEL + 0.128, WRIST_PARALLEL);
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
        }
    }
    double ARM_SPEED = 0.003;
    double WRIST_SPEED = 0.003;
    public final static double ARM_PARALLEL = 0.5; //Left Servo Parallel
    public final static double WRIST_PARALLEL = 0.5; //Wrist Servo Parallel
    double OPEN = 0.137;
    double CLOSE = 0.647;
    Servo armPivotLeft, armPivotRight;
    Servo wristServo;
    Servo claw;

    boolean isClawOpen;

    double pivRightOffset = 0.0;

    PivotArmStates currentState;

    public DepositArmV2(HardwareMap hw){
        this(hw, "depositForward", "depositBackward", "depositWrist","claw");
    }
    public DepositArmV2(HardwareMap hw, String pivLeft, String pivRight, String wrist, String clawName){
        armPivotLeft = hw.get(Servo.class,pivLeft);
        armPivotRight = hw.get(Servo.class,pivRight);
        wristServo = hw.get(Servo.class,wrist);

        armPivotRight.setDirection(Servo.Direction.REVERSE);

        claw = hw.get(Servo.class, clawName);
        DepositArm.PivotArmStates.reset.resetPositions();
    }

    public void init(){
        goToPosition(DepositArmV2.PivotArmStates.TRASNFER);
        openClaw();
    }

    public void goToPosition(DepositArmV2.PivotArmStates state){
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
    public void openClawAuto(){
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
                        "Forward Pivot Position: %f\n" +
                        "Backward Pivot Position: %f\n" +
                        "Backward Pivot Offset: %f\n" +
                        "Wrist Position: %f\n" +
                        "Claw Open: %b\n\n"+
                        "Arm From Parallel: %f\n"+
                        "Wrist From Parallel: %f\n",
                currentState,
                armPivotLeft.getPosition(),
                armPivotRight.getPosition(),
                pivRightOffset,
                wristServo.getPosition(),
                isClawOpen,
                (armPivotLeft.getPosition() - ARM_PARALLEL),
                (wristServo.getPosition() - WRIST_PARALLEL));
    }
}
