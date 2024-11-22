package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Wrist {

    public enum WristStates{
        SUB_HOVER, //Submersible Hover
        SAMPLE_INTAKE, //Sample Intake Position
        SAMPLE_DEPOSIT, //Sample Deposit Position
        SPECIMEN_INTAKE, //Perpendicular position to Intake Specimen from the Wall
        SPECIMEN_DEPOSIT, //Deposit Specimen
        PARALLEL_MODE, //Parallel position relative to ground from 0-90 degrees

        VERTICAL_POSITION

    }

    private HashMap<WristStates, Double> wristPositions;

    public final double LOWER_LIMIT = 0; // 0.25 = 90 degrees
    public final double UPPER_LIMIT = 0.55;
    public final double WRIST_PARALLEL = 0.218; // Servo position to be parallel when Arm Rotation = 0

    private double lastAngle;
    private double callsPerSecond = 50;
    private double rotationPerSecond = 0.2 / callsPerSecond; //Max of 0.04 change in servo position. Up to 100 calls per second
    private long lastCalled = 0;
    private WristStates currentState;
    private double currentPosition;

    private Servo wrist;

    //------------------------------------------------------------------------------------------
    //----------------------------------Initialization Constructors-----------------------------
    //------------------------------------------------------------------------------------------
    public Wrist(HardwareMap hw){
        this(hw, "wrist");
    }

    public Wrist(HardwareMap hw, String name){
        wrist = hw.get(Servo.class, name);
        wrist.setDirection(Servo.Direction.REVERSE);
        currentState = WristStates.SAMPLE_INTAKE;

        wristPositions = new HashMap<WristStates, Double>();


        //Intake Positions
        wristPositions.put(WristStates.SAMPLE_INTAKE, WRIST_PARALLEL - 0.015); //0.694
        wristPositions.put(WristStates.SPECIMEN_INTAKE, WRIST_PARALLEL - 0.25); //Set 90 degrees off from parallel

        //Deposit Positions
        wristPositions.put(WristStates.SAMPLE_DEPOSIT, WRIST_PARALLEL + 0.174);//0.392
        wristPositions.put(WristStates.SPECIMEN_DEPOSIT, WRIST_PARALLEL + 0.02);//0.24

        wristPositions.put(WristStates.PARALLEL_MODE, WRIST_PARALLEL);
        wristPositions.put(WristStates.SUB_HOVER, WRIST_PARALLEL + 0.12);
        wristPositions.put(WristStates.VERTICAL_POSITION, WRIST_PARALLEL);
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Set Position Functions----------------------------------
    //------------------------------------------------------------------------------------------


    /**
     * Automatically re-adjusts wrists to be at a specific angle relative to the ground.
     * @param angleToGround [double] Calculated angle between the ground to the arm.
     */
    public void wristParallelToGround(double angleToGround){
        lastAngle = angleToGround;
        if (currentState == WristStates.PARALLEL_MODE){
            double newPos = clamp(wristPositions.get(WristStates.PARALLEL_MODE) -  (angleToGround / 360), LOWER_LIMIT, UPPER_LIMIT);
            currentPosition = wristPositions.get(WristStates.PARALLEL_MODE);
            wrist.setPosition(newPos);
        }
    }

    /**
     * Goes to desired position associated with a specific state
     * @param state [WristStates] The desired state/position for the wrist
     */
    public void goToPosition(WristStates state){
        currentState = state;
        if (state != WristStates.PARALLEL_MODE) {
            wrist.setPosition(wristPositions.get(state));
        }
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    public double getWristPosition(){
        return wrist.getPosition();
    }

    public WristStates getState(){
        return currentState;
    }

    //------------------------------------------------------------------------------------------
    //----------------------------Tuning/Tweaking Functions-------------------------------------
    //------------------------------------------------------------------------------------------
    // For manual testing
    public void setPosition(double power){
        long timeNow = System.currentTimeMillis();
        if (timeNow < (lastCalled + (1000/callsPerSecond))){
            return;
        }
        lastCalled = timeNow;
        double newPos = wristPositions.get(currentState) + (power * rotationPerSecond);

        wristPositions.put(currentState, newPos);

        if (currentState != WristStates.PARALLEL_MODE) {
            wrist.setPosition(newPos);
        }
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    private double clamp(double value, double min, double max){
        return Math.max( min , Math.min( max , value));
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format(
                "Wrist Position: %f\n" +
                "Wrist State: %s\n" +
                "Wrist current position: %f\n",
                getWristPosition(),
                currentState,
                currentPosition

        );
    }
}
