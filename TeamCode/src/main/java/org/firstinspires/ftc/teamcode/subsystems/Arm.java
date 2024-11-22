package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.HashMap;

public class Arm {

    //Adjustable Constants
    private double ARM_SPEED = 50; //Ticks

    private double TICKS_PER_ROTATION = 8192; //Ticks

//    private double MINIMUM_ROTATION = 0; //0 degrees (Relative to starting position)
//    private double MAXIMUM_ROTATION = 3000; //Ticks
//
//    private int BASE_HEIGHT = 3400;//3450 - 50; //ticks
//    private int SPECIMEN_HEIGHT = 2420;//3450 - 800; //ticks
//
//    private int REST_HEIGHT = 200;//3450 - 3250; //ticks
//    private int DEPOSIT_HEIGHT = -220;//3450 - 3670; //ticks 3670

    public enum ArmState{
        SAMPLE_INTAKE, //Position to intake Samples
        SAMPLE_DEPOSIT, //Deposit Sample (Backward)
        SAMPLE_DEPOSIT_FORWARD, //Deposit Sample Forward
        SPECIMEN_DEPOSIT, //Deposit Specimen
        SPECIMEN_INTAKE, //Position to intake Specimen
        STOW_POSITION, //Default Stowed Position
        VERTICAL_POSITION //Align Arm with Lift
    }

    private HashMap<ArmState, Integer> armPositions;

    //Degrees from +x axis (where x axis is the ground and positive is the robot's forward)
    private double armAngleOffset = 142;

    //Internal variables
    private CRServo armLeft, armRight;
    private ArmState currentState;


    private Encoder encoder;
    private DcMotorEx motorPort;
    private PIDController pid;
    private int targetPosition;

    public Arm(HardwareMap hw){
        this(hw, "armLeft", "armRight", "FLM");
    }
    public Arm(HardwareMap hw, String nameLeft, String nameRight, String nameEncoder){
        //Initialize hardware
        armLeft = hw.get(CRServo.class, nameLeft);
        armRight = hw.get(CRServo.class, nameRight);
        motorPort = hw.get(DcMotorEx.class, nameEncoder);
        //Resets encoder on start
        motorPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPort.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Creates encoder object to use
        encoder = new OverflowEncoder(new RawEncoder(motorPort));

        //Reverse Directions for Right Servo
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Pid Setup
        pid = new PIDController(0.0005,0,0.00006,0.11);
        pid.setTarget(getPosition());
        currentState = ArmState.STOW_POSITION;

        armPositions = new HashMap<ArmState,Integer>();

        //Deposit Positions
        armPositions.put(ArmState.SAMPLE_DEPOSIT, -220);
        armPositions.put(ArmState.SPECIMEN_DEPOSIT, -90);
        armPositions.put(ArmState.SAMPLE_DEPOSIT_FORWARD, 2000);

        //Intake Positions
        armPositions.put(ArmState.SPECIMEN_INTAKE, 3400);
        armPositions.put(ArmState.SAMPLE_INTAKE,3170);

        //Stow Positions
        armPositions.put(ArmState.STOW_POSITION, 200);
        armPositions.put(ArmState.VERTICAL_POSITION,1000);

    }


    //------------------------------------------------------------------------------------------
    //----------------------------------Go To Position----------------------------------------
    //------------------------------------------------------------------------------------------


    public void goToPosition(ArmState state){
        currentState = state;
        targetPosition = armPositions.get(state);
        pid.setTarget(targetPosition);
    }


    //------------------------------------------------------------------------------------------
    //----------------------------------Getter Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    public int getTargetPosition(){
        return targetPosition;
    }

    public int getPosition(){
        return motorPort.getCurrentPosition();//encoder.getPositionAndVelocity().position;
    }
    public double getVelocity() {
        return motorPort.getVelocity();//encoder.getPositionAndVelocity().velocity;
    }

    public double getForwardFeedValue(){
        return -Math.cos(Math.toRadians(getRotation()));
    }


    /**
     * @return [double] The rotational position in DEGREES relative to ground
     */
    public double getRotation(){
        //Initial start position is ~=-50 degrees off of +y axis
        return armAngleOffset - ((getPosition() / TICKS_PER_ROTATION) * 360);
    }

    public PIDController getPid(){
        return pid;
    }
    public ArmState getState(){
        return currentState;
    }
    //------------------------------------------------------------------------------------------
    //----------------------------------PID Functions-------------------------------------------
    //------------------------------------------------------------------------------------------
    /**
     * This ensures that the PID loops properly.
     * Please add this into the TeleOp loop when using this.
     */
    public double update(){
        double power = pid.calculate(getPosition(), getForwardFeedValue());
        armLeft.setPower(power);
        armRight.setPower(power);
        return power;

        
    }

    /**
     * Increases/decreases a PID tuning value by a set amount
     * @param Kp [double] Increment to increase Kp by
     * @param Ki [double] Increment to increase Ki by
     * @param Kd [double] Increment to increase Kd by
     */

    //------------------------------------------------------------------------------------------
    //----------------------------------Tuning/Tweaking Functions-------------------------------
    //------------------------------------------------------------------------------------------

    public void adjustPID(double Kp, double Ki, double Kd, double Kf){
        pid.setKp(Kp);
        pid.setKi(Ki);
        pid.setKd(Kd);
        pid.setKf(Kf);
    }
    public void setPosition(double power){
        targetPosition +=  (power * ARM_SPEED);
        pid.setTarget(targetPosition);
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions----------------------------------------
    //------------------------------------------------------------------------------------------
    private int clamp(double value, double max, double min){
        return (int) Math.max( min , Math.min( max , value));
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        return String.format("Arm current Rotation: %f\n" +
                "Arm current position: %d\n" +
                "Arm target position: %d\n" +
                "Arm PID Data: \n%s",
                this.getRotation(),
                this.getPosition(),
                this.targetPosition,
                pid.toString());
    }

    @SuppressLint("DefaultLocale")
    public String toString(boolean disablePID){
        if(disablePID){
            return String.format(
                    "Arm current Rotation: %f\n" +
                    "Arm current position: %d\n" +
                    "Arm target position: %d\n" +
                    "Arm State: %s\n",
                    this.getRotation(),
                    this.getPosition(),
                    this.targetPosition,
                    this.currentState);
        }
        return toString();
    }

    //------------------------------------------------------------------------------------------
    //----------------------------------Autonomous Functions------------------------------------
    //------------------------------------------------------------------------------------------
    public Action armPID(){
        return new ArmPID();
    }

    public class ArmPID implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }

}
