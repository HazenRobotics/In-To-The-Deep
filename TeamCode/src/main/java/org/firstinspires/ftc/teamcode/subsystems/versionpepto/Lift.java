package org.firstinspires.ftc.teamcode.subsystems.versionpepto;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.HashMap;

public class Lift {

    //Adjustable Constants
    public int LIFT_SPEED = 50; //ticks per call



    //Lift Positions and possible States

    public enum LiftStates{
        SAMPLE_DEPOSIT, //Maximum Possible Height for Sample Scoring
        SAMPLE_INTAKE, // Lift at Submersible Intake Height

        SPECIMEN_DEPOSIT, //Specimen Scoring Height
        SPECIMEN_DEPOSIT_FORWARD, //Specimen Forward Scoring Height
        SPECIMEN_INTAKE, //Specimen Intake Height
        ZERO, //Lift at Lowest Possible Height
        CLIMB, //Preparing to Climb

        CLEARANCE_FOR_ULTRASONIC //Raise lift for ultra sonic clearance

    }

    private int MAX_HEIGHT_POSITION = 2300;//ticks
    private LiftStates currentState;

    private HashMap<LiftStates, Integer> liftPositions;


    //Was planning to tune weight, but just stuck with the mass of 1 kg
    public double calculatedWeight = 9.8;

    //Internal variables
    private final DcMotorEx liftLeft, liftRight;

    private int targetPosition;

    private int liftOffset = 0;

    private HardwareMap hw;
    private final Encoder encoder;
    private String encoderName;
    private final PIDController pid;

    //Controls/enables the PID to be activated or not, primarily for autonomous
    private boolean autoPIDActive = true;

    //-----------------------------------------------------------------------------------------
    //----------------------------------Initialization-----------------------------------------
    //-----------------------------------------------------------------------------------------
    /**
     * Quick Constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize the motors.
     *///
    public Lift(HardwareMap hw){
        this(hw, "liftLeft", "liftRight", "FRM");
    }

    /**
     * Primary constructor for the Lift Subsystem Class
     * @param hw [HardwareMap] Hardware map necessary to initialize motors.
     * @param nameLeft [String] Name of the left motor assigned in the configuration.
     * @param nameRight [String] Name of the right motor assigned in the configuration.
     */
    public Lift(HardwareMap hw, String nameLeft, String nameRight, String encoderName){
        this.hw = hw;
        this.encoderName = encoderName;

        //Initialize motors
        this.liftLeft = hw.get(DcMotorEx.class, nameLeft);
        this.liftRight = hw.get(DcMotorEx.class, nameRight);

        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, encoderName)));
        //Reverse one of the motors
        this.liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        currentState = LiftStates.ZERO;


        pid = new PIDController(0.009,0,0.00002,0.017);
        pid.setTarget(getPosition());

        liftPositions = new HashMap<LiftStates, Integer>(3);
        //Calculate the ratios by [Desired Tick Position] / [MAX_HEIGHT_POSITION]
        //This way the ratio can be reused despite different gear ratios or tick resolutions
        //Aim for at least 6 decimal places, the more the better.


        setLiftPositions();
    }

    public void setLiftPositions(){
        //Intake Positions
        liftPositions.put(LiftStates.SAMPLE_INTAKE, (int) (0.0163934 * MAX_HEIGHT_POSITION));
        liftPositions.put(LiftStates.SPECIMEN_INTAKE,(int) (0.31860465 * MAX_HEIGHT_POSITION));

        //Deposit Positions
        liftPositions.put(LiftStates.SPECIMEN_DEPOSIT, (int) (0.525581 * MAX_HEIGHT_POSITION));
        liftPositions.put(LiftStates.SAMPLE_DEPOSIT, (int) (0.99 * MAX_HEIGHT_POSITION));
        liftPositions.put(LiftStates.SPECIMEN_DEPOSIT_FORWARD,(int) (0.233558 * MAX_HEIGHT_POSITION));

        //Climb/Stow Positions
        liftPositions.put(LiftStates.ZERO, 0);
        liftPositions.put(LiftStates.CLIMB, (int) (0.524590 * MAX_HEIGHT_POSITION));

        liftPositions.put(LiftStates.CLEARANCE_FOR_ULTRASONIC, (int) (0.069767 * MAX_HEIGHT_POSITION));
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
        targetPosition = liftPositions.get(state) + liftOffset;
        pid.setTarget(targetPosition);
    }


    //------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    /**
     * Tester function to track current height.
     * @return [int] Returns current position of the left motor in ticks.
     */
    public int getPosition(){
        return encoder.getPositionAndVelocity().position;
    }
    public int getVelocity() {return encoder.getPositionAndVelocity().velocity;}

    public int getTargetPosition(){
        return targetPosition;
    }
    public LiftStates getState(){
        return currentState;
    }

    public double getForwardFeedValue(){
        return calculatedWeight;
    }

    public PIDController getPid(){
        return pid;
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
        pid.setKp(Kp);
        pid.setKi(Ki);
        pid.setKd(Kd);
        pid.setKf(Kf);
    }


    /**
     * Manual tester to adjust the height of the lift.
     * @param power [double] Can be paired with a joystick or trigger to have a dynamic speed to raise the lift.
     * @return [int] Returns the new target position of the lift in ticks.
     */
    public int setPosition(double power){
        targetPosition += (power * LIFT_SPEED);
        liftPositions.put(currentState,targetPosition);
        pid.setTarget(targetPosition);
        return targetPosition;
    }

    /**
     * Updates PID loop/motor power.
     * Ensure this is called when using lift, otherwise nothing will happen.
     */
    public double update(){
        //Default power will be the Forward Feed Constant
        double power = pid.getPIDValues()[3] * getForwardFeedValue();
        if (autoPIDActive) {
            power = pid.calculate(getPosition(), getForwardFeedValue());
        }
        liftLeft.setPower(power);
        liftRight.setPower(power);
        return power;
    }

    public void doubleCurrentReset(){
        double currentLimit = 10;
        if (liftLeft.getCurrent(CurrentUnit.AMPS) + liftRight.getCurrent(CurrentUnit.AMPS) > currentLimit
        && targetPosition < 0){
            resetLift();
        }
    }

    /**
     * Resets the lift Encoders
     */
    public void resetLift(){
        DcMotorEx port = hw.get(DcMotorEx.class,encoderName);
        port.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentState = LiftStates.ZERO;
        targetPosition = 0;
    }

    public void resetLiftOffset(){
        liftOffset = encoder.getPositionAndVelocity().position;
        setLiftPositions();
    }

    //-----------------------------------------------------------------------------------------
    //----------------------------------Helper Functions---------------------------------------
    //-----------------------------------------------------------------------------------------
    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format(
                "Lift current position: %d\n" +
                "Lift target position: %d\n" +
                "Lift PID Data: \n%s",
                getPosition(),
                targetPosition,
                pid.toString());
    }

    @SuppressLint("DefaultLocale")
    public String toString(boolean disablePID){
        if (disablePID){
            return String.format(
                    "Lift current position: %d\n" +
                    "Lift target position: %d\n" +
                    "Lift current state: %s\n" +
                    "%s\n",
                    getPosition(),
                    targetPosition,
                    currentState,
                    getLiftCurrent());
        }
        return toString();
    }

    @SuppressLint("DefaultLocale")
    public String getLiftCurrent(){
        return String.format(
                "Left Arm Current: %f\n" +
                        "Right Arm Current: %f",
                liftLeft.getCurrent(CurrentUnit.AMPS),
                liftRight.getCurrent(CurrentUnit.AMPS)
        );
    }


    private double clamp(double value, double max, double min){
        return Math.max( min , Math.min( max , value));
    }

    //-----------------------------------------------------------------------------------------
    //----------------------------------Autonomous Actions-------------------------------------
    //-----------------------------------------------------------------------------------------
    public Action liftPID(){
        return new LiftPID();
    }

    public void setAutoPIDActive(boolean active){
        autoPIDActive = active;
    }

    public class LiftPID implements  Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return autoPIDActive;
        }
    }


}
