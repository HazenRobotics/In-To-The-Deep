package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.versionpepto.Lift;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.HashMap;

public class DepositLift extends PIDController {

    private static final int LIFT_SPEED = 25;

    //CHANGE THESE VARIABLES
    public void resetLiftOffset(){
        liftOffset = encoder.getPositionAndVelocity().position;
        LiftStates.TRANSFER.setPosition(0);
        LiftStates.SPECIMEN_INTAKE.setPosition(54);
        LiftStates.SPECIMEN_DEPOSIT.setPosition(205);
        LiftStates.SAMPLE_DEPOSIT.setPosition(720);
        LiftStates.SPECIMEN_INTAKE_RAISE.setPosition(LiftStates.SPECIMEN_INTAKE.getPosition() + 150);
    }

    //---------------------------------DO NOT TOUCH-------------------------------------------------
    //------------------------------THESE CHANGE NOTHING--------------------------------------------
    public enum LiftStates {

        TRANSFER(0), //Default Position
        SPECIMEN_INTAKE(54), //Specimen Intake Position
        SPECIMEN_DEPOSIT(205), //Specimen Deposit Position
        SAMPLE_DEPOSIT(720),// Sample Deposit Position
        SPECIMEN_DEPOSIT_PRELOAD(415),
        SPECIMEN_INTAKE_AUTO(200),
        SPECIMEN_INTAKE_RAISE(SPECIMEN_INTAKE.getPosition() + 80),

        SPECIMEN_INTAKE_SIDEWAYS(210),
        SPECIMEN_DEPOSIT_SIDEWAYS(250);




        double position;
        LiftStates(double position){
            this.position = position;
        }


        public double getPosition() {
            return position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }

    LiftStates currentState;

    DcMotorEx liftLeft, liftRight;
    Encoder encoder;


    private int liftOffset;
    //----------------------------------------------------------------------------------------------
    //----------------------------------Initialization----------------------------------------------
    //----------------------------------(Constructors)----------------------------------------------

    public DepositLift(HardwareMap hw){
        this(hw, "liftLeft", "liftRight","FLM");
    }

    public DepositLift(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        super(0.025,0.0,0.001,0.0);
        liftLeft = hw.get(DcMotorEx.class, nameLeft);
        liftRight = hw.get(DcMotorEx.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));

        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        liftOffset = 0;
        currentState = LiftStates.TRANSFER;
    }


    //-----------------------------------------------------------------------------------------
    //----------------------------------Go To Positions----------------------------------------
    //-----------------------------------------------------------------------------------------


    /**
     * This will cause the lift to travel to the given LiftState.
     * @param state [LiftStates] - Sets the lift to go to the given Lift Position State.
     */
    public void goToPosition(LiftStates state){
        currentState = state;
        super.setTarget(state.getPosition() + liftOffset);
        super.setTarget(super.getTarget());
    }

    //----------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions--------------------------------------------
    //----------------------------------------------------------------------------------------------

    public double getForwardFeed(){
        return 1;
    }

    public LiftStates getCurrentState() {
        return currentState;
    }

    public int getPosition(){
        return encoder.getPositionAndVelocity().position;
    }
    public int getVelocity(){
        return encoder.getPositionAndVelocity().velocity;
    }

    /**
     * Updates PID loop/motor power.
     * Ensure this is called when using lift, otherwise nothing will happen.
     */
    public double updatePID(){
        double power = super.calculate(getPosition(), getForwardFeed());
        liftLeft.setPower(power);
        liftRight.setPower(power);
        return power;
    }
    //-----------------------------------------------------------------------------------------
    //----------------------------------Tune/Tweak Functions-----------------------------------
    //-----------------------------------------------------------------------------------------
    /**
     * Increases/decreases a PID tuning value by a set amount
     * @param Kp [double] Increment to increase Kp by
     * @param Ki [double] Increment to increase Ki by
     * @param Kd [double] Increment to increase Kd by
     */
    public void adjustPID(double Kp, double Ki, double Kd, double Kf){
        super.setKp(Kp);
        super.setKi(Ki);
        super.setKd(Kd);
        super.setKf(Kf);
    }


    /**
     * Manual tester to adjust the height of the lift.
     * @param power [double] Can be paired with a joystick or trigger to have a dynamic speed to raise the lift.
     * @return [int] Returns the new target position of the lift in ticks.
     */
    public int setPosition(double power){
        int targetPosition = (int) (super.getTarget() + (power * LIFT_SPEED));
        currentState.setPosition(targetPosition);
        super.setTarget(targetPosition);
        return targetPosition;
    }

    public int setPositionInverse(double power){
        int targetPosition = (int) (super.getTarget() + (power * LIFT_SPEED));
        currentState.setPosition(targetPosition);
        super.setTarget(targetPosition);
        return targetPosition;
    }




    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Target Position: %f\n" +
                        "Current Position: %d\n" +
                        "Current State: %s\n" +
                "Current Amps Left: %f\n" +
                        "Current Amps Right: %f\n" ,
                super.getTarget(),
                getPosition(),
                getCurrentState(),
                liftLeft.getCurrent(CurrentUnit.AMPS),
                liftRight.getCurrent(CurrentUnit.AMPS));
    }

    public String toStringPID(){
        return super.toString();
    }

}
