package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.MiscMethods;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.HashMap;


public class ExtendoSlide extends PIDController {

    double EXTENDO_SPEED = 40;

    static double MAX_EXTENSION = 400;
    public enum ExtendoStates{
        TRANSFER(0),
        HALF_EXTEND(MAX_EXTENSION/2),
        FULL_EXTEND(MAX_EXTENSION),

        VARIABLE_EXTEND(TRANSFER.getPosition() + 50);

        private double position;
        ExtendoStates(double pos){
            position = pos;
        }
        public double getPosition(){
            return position;
        }
        void setPosition(double pos){
            position = pos;
        }
    }

    DcMotorEx extendo;
    Encoder encoder;

    DigitalChannel bumpSwitch;

    HashMap<ExtendoStates, Integer> extendoPositions;

    ExtendoStates currentState;
    double extendoOffset;


    //----------------------------------------------------------------------------------------------
    //----------------------------------Initialization----------------------------------------------
    //----------------------------------(Constructors)----------------------------------------------
    public ExtendoSlide(HardwareMap hw, String nameMotor, String nameEncoder){
        super(0.03,0,0,0);
        extendo = hw.get(DcMotorEx.class, nameMotor);
        //Initially negative, but I want to work with positives only
//        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));
//        encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        extendoOffset = 0;
        currentState = ExtendoStates.TRANSFER;



        //Addition of new reset feature
        bumpSwitch = hw.digitalChannel.get("bumpSwitch");
    }

    public ExtendoSlide(HardwareMap hw){
        this(hw, "extendo","BRM");
    }


    //-----------------------------------------------------------------------------------------
    //----------------------------------Go To Positions----------------------------------------
    //-----------------------------------------------------------------------------------------


    /**
     * This will cause the lift to travel to the given LiftState.
     * @param state [LiftStates] - Sets the lift to go to the given Lift Position State.
     */
    public void goToPosition(ExtendoStates state){
        currentState = state;
        super.setTarget(state.getPosition() + extendoOffset);
        super.setTarget(super.getTarget());
    }

    //----------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions--------------------------------------------
    //----------------------------------------------------------------------------------------------

    public double getForwardFeed(){
        return 1;
    }

    public ExtendoStates getCurrentState() {
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
        extendo.setPower(power);
//        if (extendo.getCurrent(CurrentUnit.AMPS) > 5 && Math.abs(getVelocity()) < 5){
//            extendoOffset = getPosition();
//        }
        if (!bumpSwitch.getState()){
            resetExtendoOffset();
        }
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
        int targetPosition = (int) MiscMethods.clamp(super.getTarget() + (power * EXTENDO_SPEED), extendoOffset, MAX_EXTENSION);
//        currentState.setPosition(targetPosition);
        if (extendoOffset + 50 < targetPosition){
            currentState = ExtendoStates.VARIABLE_EXTEND;
        }else{
           currentState = ExtendoStates.TRANSFER;
        }
        super.setTarget(targetPosition);
        return targetPosition;
    }
    public void setPositionInverse(double power){
        int targetPosition = (int) Math.min(super.getTarget() + (power * EXTENDO_SPEED), MAX_EXTENSION);
        ExtendoStates.TRANSFER.setPosition(targetPosition);
        extendoOffset = targetPosition;
        super.setTarget(targetPosition);
    }

    public int setPositionRestricted(double power){
        int targetPosition = (int) MiscMethods.clamp(super.getTarget() + (power * EXTENDO_SPEED), extendoOffset ,MAX_EXTENSION);
//        currentState.setPosition(targetPosition);
        if (extendoOffset + targetPosition > 50){
            currentState = ExtendoStates.VARIABLE_EXTEND;
        }else{
            currentState = ExtendoStates.TRANSFER;
        }
        super.setTarget(targetPosition);
        return targetPosition;
    }


    public void resetExtendoOffset(){
        extendoOffset = encoder.getPositionAndVelocity().position;
        ExtendoStates.TRANSFER.setPosition(0);
        ExtendoStates.HALF_EXTEND.setPosition(MAX_EXTENSION/2);
        ExtendoStates.FULL_EXTEND.setPosition(MAX_EXTENSION);
    }

    public double getExtendoOffset(){
        return extendoOffset;
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Target Position: %f\n" +
                "Current Position: %d\n" +
                "Current State: %s\n"+
                "Motor Current %f\n",
                super.getTarget(),
                getPosition(),
                getCurrentState(),
                extendo.getCurrent(CurrentUnit.AMPS));
    }

    public String toStringPID(){
        return super.toString();
    }
}
