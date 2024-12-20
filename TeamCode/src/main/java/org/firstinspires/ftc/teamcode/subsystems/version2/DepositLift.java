package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.versionpepto.Lift;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.HashMap;

public class DepositLift extends PIDController {

    private static final int LIFT_SPEED = 50;


    public enum LiftStates {
        ZERO
    }

    LiftStates currentState;

    DcMotorEx liftLeft, liftRight;
    Encoder encoder;

    private HashMap<LiftStates, Integer> liftPositions;

    private int liftOffset;
    //----------------------------------------------------------------------------------------------
    //----------------------------------Initialization----------------------------------------------
    //----------------------------------(Constructors)----------------------------------------------

    public DepositLift(HardwareMap hw){
        this(hw, "liftLeft", "liftRight","liftLeft");
    }

    public DepositLift(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        super(0,0,0,0);
        liftLeft = hw.get(DcMotorEx.class, nameLeft);
        liftRight = hw.get(DcMotorEx.class, nameRight);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, nameEncoder)));

        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        liftOffset = 0;
        initializePositions();
    }

    public void initializePositions(){
        liftPositions = new HashMap<LiftStates,Integer>();
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
        super.setTarget(liftPositions.get(state) + liftOffset);
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

    /**
     * Updates PID loop/motor power.
     * Ensure this is called when using lift, otherwise nothing will happen.
     */
    public void updatePID(){
        double power = super.calculate(encoder.getPositionAndVelocity().position, getForwardFeed());
        liftLeft.setPower(power);
        liftRight.setPower(power);
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
        liftPositions.put(currentState, targetPosition);
        super.setTarget(targetPosition);
        return targetPosition;
    }

    public void resetLiftOffset(){
        liftOffset = encoder.getPositionAndVelocity().position;
        initializePositions();
    }

}
