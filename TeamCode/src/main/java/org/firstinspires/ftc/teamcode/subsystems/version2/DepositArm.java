package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MiscMethods;

public class DepositArm {

    double ARM_SPEED = 0.003;
    double WRIST_SPEED = 0.003;
    public final static double ARM_PARALLEL = 0.483; //Left Servo Parallel
    public final static double WRIST_PARALLEL = 0.643; //Wrist Servo Parallel

    double OPEN = 0.5;
    double OPEN_AUTO = 0.75;
    double CLOSE = 1;

    public enum PivotArmStates{
        reset(0,0),
        TRASNFER(ARM_PARALLEL - 0.2, WRIST_PARALLEL - 0.521),
        SPECIMEN_INTAKE(ARM_PARALLEL + 0.252, WRIST_PARALLEL + 0.315),
        SPECIMEN_DEPOSIT(ARM_PARALLEL - 0.119, WRIST_PARALLEL - 0.173),
        SAMPLE_DEPOSIT(ARM_PARALLEL + 0.004, WRIST_PARALLEL + 0.182),
        SPECIMEN_DEPOSIT_PRELOAD(ARM_PARALLEL-0.29,WRIST_PARALLEL),
        SPECIMEN_INTAKE_AUTO(ARM_PARALLEL + 0.295, WRIST_PARALLEL + 0.283),

        SPECIMEN_INTAKE_SIDEWAYS(ARM_PARALLEL + 0.517, WRIST_PARALLEL -0.237),
        SPECIMEN_DEPOSIT_SIDEWAYS(ARM_PARALLEL - 0.117, WRIST_PARALLEL - 0.25),
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
            TRASNFER.setArmWrist(ARM_PARALLEL - 0.2, WRIST_PARALLEL - 0.521);//0.272, 0.131
            SPECIMEN_INTAKE.setArmWrist(ARM_PARALLEL + 0.252, WRIST_PARALLEL + 0.315);//0.798,0.928
            SPECIMEN_DEPOSIT.setArmWrist(ARM_PARALLEL - 0.119, WRIST_PARALLEL - 0.173);//0.351, 0.511
            SAMPLE_DEPOSIT.setArmWrist(ARM_PARALLEL + 0.004, WRIST_PARALLEL + 0.182);
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
