package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.HashMap;


public class ExtendoSlide extends PIDController {

    public enum ExtendoStates{
        ZERO
    }
    DcMotorEx extendo;
    Encoder encoder;

    double EXTENDO_SPEED;

    HashMap<ExtendoStates, Integer> extendoPositions;

    ExtendoStates currentState;
    double extendoOffset;


    //----------------------------------------------------------------------------------------------
    //----------------------------------Initialization----------------------------------------------
    //----------------------------------(Constructors)----------------------------------------------
    public ExtendoSlide(HardwareMap hw, String nameMotor, String nameEncoder){
        super(0,0,0,0);
        extendo = hw.get(DcMotorEx.class, nameMotor);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));

        extendoOffset = 0;
        initializePositions();
    }

    public ExtendoSlide(HardwareMap hw){
        this(hw, "extendo","extendo");
    }

    public void initializePositions(){
        extendoPositions = new HashMap<ExtendoStates,Integer>();
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
        super.setTarget(extendoPositions.get(state) + extendoOffset);
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

    /**
     * Updates PID loop/motor power.
     * Ensure this is called when using lift, otherwise nothing will happen.
     */
    public void updatePID(){
        double power = super.calculate(getPosition(), getForwardFeed());
        extendo.setPower(power);
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
        int targetPosition = (int) (super.getTarget() + (power * EXTENDO_SPEED));
        extendoPositions.put(currentState, targetPosition);
        super.setTarget(targetPosition);
        return targetPosition;
    }

    public void resetLiftOffset(){
        extendoOffset = encoder.getPositionAndVelocity().position;
        initializePositions();
    }


    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Target Position: %f\n" +
                "Current Position: %d\n" +
                "Current State: %s",
                super.getTarget(),
                getPosition(),
                getCurrentState());
    }

    public String toStringPID(){
        return super.toString();
    }
}
