package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.ParkFlag;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class FourEyesRobot extends Mecanum {
    //---------------------------------------------------------------------------------------------
    //----------------------------------Subsystem Objects------------------------------------------
    //---------------------------------------------------------------------------------------------
    HardwareMap hardwareMap;
    public Lift lift;
    public Arm arm;
    public Wrist wrist;
    Claw claw;

    ParkFlag parkFlag;
    public ActiveIntake activeIntake;
    //---------------------------------------------------------------------------------------------
    //----------------------------------Internal States--------------------------------------------
    //---------------------------------------------------------------------------------------------
    enum ScoringType{
        SAMPLE,
        SPECIMEN
    }

    ScoringType currentState;

    private boolean wristAutoPIDActive = true;
    //---------------------------------------------------------------------------------------------
    //----------------------------------Initialization---------------------------------------------
    //---------------------------------------------------------------------------------------------
    public FourEyesRobot(HardwareMap hw) {
        super(hw);
        //Reassigned here to ensure that they are properly initialized
        hardwareMap = hw;
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        parkFlag = new ParkFlag(hardwareMap);
        activeIntake = new ActiveIntake(hardwareMap);
        currentState = ScoringType.SAMPLE;
    }

    /**
     * This is to provide power to servos DURING
     * the begining of START PHASE
     */
    public void initializePowerStates(){
        lift.goToPosition(Lift.LiftStates.ZERO);
        wrist.goToPosition(Wrist.WristStates.PARALLEL_MODE);
        activeIntake.deactivateIntake();
        claw.closeClaw();
        parkFlag.stowFlag();
    }

    //---------------------------------------------------------------------------------------------
    //----------------------------------Automated Controls-----------------------------------------
    //---------------------------------------------------------------------------------------------

    /**
     * Function to set up subsystems to:
     * Preparation to intake SAMPLE from the Submersible
     */
    public void intakeSamplePos() {
        lift.goToPosition(Lift.LiftStates.SAMPLE_INTAKE);//Set the lift just high enough to be above submersible
        arm.goToPosition(Arm.ArmState.SAMPLE_INTAKE);
        wrist.goToPosition(Wrist.WristStates.SUB_HOVER);
        activeIntake.deactivateIntake();
        claw.closeClaw();
        currentState = ScoringType.SAMPLE;
    }

    /**
     * Function to set up subsystems to:
     * Preparation to intake SPECIMEN from the Human Player Wall
     */
    public void intakeSpecimenPos(){
        lift.goToPosition(Lift.LiftStates.SPECIMEN_INTAKE);//Lower lift as low as possible
        arm.goToPosition(Arm.ArmState.SPECIMEN_INTAKE);//Use arm to go to an angle to decrease extention length from center of rotation
        wrist.goToPosition(Wrist.WristStates.SPECIMEN_INTAKE);//Use wrist to counter act arm's rotation
        claw.closeClaw(); //Close the claw before hand so if the arm is behind, the claw won't hit the lift
        currentState = ScoringType.SPECIMEN;
    }

    /**
     * This is to deposit samples from behind the robot.
     */
    public void depositSamplePos(){
        lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT);//Raises lift to maximum height
        arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT);//Flips arm to go backwards
        wrist.goToPosition(Wrist.WristStates.SAMPLE_DEPOSIT);//Flips wrist to angle
        claw.closeClaw(); //Closes claw if it was open from before
        currentState = ScoringType.SAMPLE;
    }

    /**
     * This is to deposit samples from the forward direction.
     */
    public void depositSamplePosForward(){
        lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT);//Raises lift to maximum height
        arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD);//Flips arm to go backwards
        wrist.goToPosition(Wrist.WristStates.PARALLEL_MODE);//Flips wrist to angle
        claw.closeClaw(); //Closes claw if it was open from before
        currentState = ScoringType.SAMPLE;
    }

    /**
     * Function to set up subsystems to:
     * Deposit SPECIMEN into High Bar
     */
    public void depositSpecimenPos(){
        lift.goToPosition(Lift.LiftStates.SPECIMEN_DEPOSIT);
//        arm.goToPosition(Arm.ArmState.BASE_HEIGHT);
//        wrist.goToPosition(Wrist.WristStates.ParallelMode);
        arm.goToPosition(Arm.ArmState.SPECIMEN_DEPOSIT);
        wrist.goToPosition(Wrist.WristStates.SPECIMEN_DEPOSIT);
        claw.closeClaw();
        currentState = ScoringType.SPECIMEN;
    }

    public void depositSpecimenPosForward(){
        lift.goToPosition(Lift.LiftStates.SPECIMEN_DEPOSIT_FORWARD);
        arm.goToPosition(Arm.ArmState.SPECIMEN_DEPOSIT_FORWARD);
        wrist.goToPosition(Wrist.WristStates.SPECIMEN_DEPOSIT_FORWARD);
        claw.closeClaw();
        currentState = ScoringType.SPECIMEN;
    }

    /**
     * Used for auto to temporarily stow Arm
     */
    public void VerticalArm(){
        arm.goToPosition(Arm.ArmState.VERTICAL_POSITION);
        wrist.goToPosition(Wrist.WristStates.VERTICAL_POSITION);
    }

    /**
     * Used for auto to set Lift to 0
     */
    public void liftGoToZero(){
        lift.goToPosition(Lift.LiftStates.ZERO);
    }

    /**
     * Reverses Intake to Deposit
     */
    public void intakeBackward() {
        activeIntake.reverseIntake();
    }
    public void intakeStop() {
//        wrist.goToPosition(Wrist.WristStates.PARALLEL_MODE);
        activeIntake.deactivateIntake();
    }


    //Right bumper
    /**
     * If the lift is currently in the Sample Intake State,
     */
    public void toggleIntake(){
        if(currentState == ScoringType.SAMPLE){
            //Sample Modes
            //Currently hovering above sub
            if (wrist.getState() == Wrist.WristStates.SUB_HOVER) {
                //Switch to intake mode
                wrist.goToPosition(Wrist.WristStates.SAMPLE_INTAKE);
                //Activate intake
                activeIntake.activateIntake();
            }
            else if(wrist.getState() == Wrist.WristStates.SAMPLE_INTAKE){
                //Switch to Hover mode
                wrist.goToPosition(Wrist.WristStates.SUB_HOVER);
                //Activate intake
                activeIntake.activateIntake();
            }
        }
    }

    //Left bumper
    /**
     * Toggles a deposit system depending on which
     * scoring system is currently active
     */
    public void toggleDeposit(){
        switch (currentState){
            //Toggles Active Intake if Sample scoring is active
            case SAMPLE:
                if (activeIntake.isRunning()) {
                    activeIntake.deactivateIntake();
                }
                else{
                    activeIntake.reverseIntake();
                }
                break;
            //Toggles claw if Specimen scoring is active
            case SPECIMEN:
                claw.toggleClaw();
                break;
            default:
                break;
        }
    }


    /**
     * Raises the lift to climb the first bar.
     */
    public void raiseClimb(){
        arm.goToPosition(Arm.ArmState.STOW_POSITION);
        lift.goToPosition(Lift.LiftStates.CLIMB);
    }

    /**
     * Function used to stow subsystems and to lower climb
     */
    public void lowerClimb(){
        arm.goToPosition(Arm.ArmState.STOW_POSITION);
        lift.goToPosition(Lift.LiftStates.ZERO);
        wrist.goToPosition(Wrist.WristStates.PARALLEL_MODE);
        claw.closeClaw();
    }


    /**
     * Required for all OpModes utilizing subsystems
     * This must be called otherwise subsystem PIDs
     * and values will not update properly
     */
    public void updatePID(){
        lift.update();
        arm.update();
        wrist.wristParallelToGround(arm.getRotation());
        activeIntake.checkForSample();
        parkFlag.wave();
    }


    public void depositBasket(){
        currentState = ScoringType.SAMPLE;
        wrist.goToPosition(Wrist.WristStates.SAMPLE_DEPOSIT);
    }

    //---------------------------------------------------------------------------------------------
    //----------------------------------Manual Controls--------------------------------------------
    //---------------------------------------------------------------------------------------------
    public boolean isIntaking() {
        return activeIntake.isRunning();
    }
    public void deactivateIntake(){
        activeIntake.deactivateIntake();
    }
    public void activateIntake(){
        activeIntake.activateIntake();
    }

    public void openClaw() {
        claw.openClaw();
    }

    public void closeClaw(){
        claw.closeClaw();
    }

    public void openWideClaw(){
        claw.openClawWide();
    }

    public void moveLift(double power) {
        lift.setPosition(power);
    }
    public void changeHeightArm(double power) {
        arm.setPosition(power);
    }
    public void setWristPosition(double power){
        wrist.setPosition(power);
    }

    public void raiseFlag(){
        parkFlag.raiseFlag();
    }
    public void stowFlag(){parkFlag.stowFlag();}

    public void toggleFlagWave(){parkFlag.toggleWave();}
    public void turnOnWave(){parkFlag.turnOnWave();}

    public void resetLift(){
        lift.resetLift();
    }
    public void resetArmStartOffset(){
        arm.resetArmOffset();
    }
    //---------------------------------------------------------------------------------------------
    //----------------------------------Auto Actions Controls--------------------------------------
    //---------------------------------------------------------------------------------------------
    public Action autoPID(){
        lift.setAutoPIDActive(true);
        arm.setAutoPIDActive(true);
        wristAutoPIDActive = true;
        return new ParallelAction(
                lift.liftPID(),
                arm.armPID(),
                new wristParallel(),
                new activeCheck()
        );
    }

    public InstantAction endPID(){
        return new InstantAction(() -> {
            lift.setAutoPIDActive(false);
            arm.setAutoPIDActive(false);
            wristAutoPIDActive = false;
        });
    }

    public Action waitForLiftArmPID(double seconds){
        return new WaitForLiftArmPID((long) seconds);
    }

    public Action waitForLiftArmPID(double seconds, double liftPosError, double armPosError, double liftVelError, double armVelError){
        return new WaitForLiftArmPID((long) seconds,liftPosError,armPosError,liftVelError,armVelError);
    }
    //This needed to be here since it saves the issue of transferring arm rotation to the wrist
    //class and then calling wrist to transfer a new wrist action
    public class wristParallel implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.wristParallelToGround(arm.getRotation());
            return wristAutoPIDActive;
        }
    }
    public class activeCheck implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            activeIntake.checkForSample();
            return wristAutoPIDActive;
        }
    }

    public class WaitForLiftArmPID implements Action{

        private long maxWaitSeconds;

        private double liftPos, armPos, liftVel, armVel;
        public WaitForLiftArmPID(long maxWaitSeconds){
            this.maxWaitSeconds = System.currentTimeMillis() + maxWaitSeconds * 1000;
            this.liftPos = 100;
            this.armPos = 100;
            this.liftVel = 50;
            this.armVel = 50;
        }
        public WaitForLiftArmPID(long maxWaitSeconds, double liftPos, double armPos, double liftVel, double armVel){
            this.maxWaitSeconds = System.currentTimeMillis() + maxWaitSeconds * 1000;
            this.liftPos = liftPos;
            this.armPos = armPos;
            this.liftVel = liftVel;
            this.armVel = armVel;

        }

        /**
         * Returns true if this is is supposed to loop again, returns false to stop
         * @param telemetryPacket
         * @return
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /* This run block ends once the following is true:
            This was running for more longer than the allocated maxWaitForSeconds
            OR
            Lift Position Error within 50 ticks, Lift Velocity within 20 ticks
            AND
            Arm Position Error within 50 ticks, Arm Velocity within 20 ticks
             */

            return System.currentTimeMillis() < this.maxWaitSeconds &&
                    (Math.abs(lift.getTargetPosition() - lift.getPosition()) > liftPos
                            || Math.abs(lift.getVelocity()) > armPos
                            || Math.abs(arm.getTargetPosition() - arm.getPosition()) > armPos
                            || Math.abs(arm.getVelocity()) > armVel
                            );
        }
    }


    //---------------------------------------------------------------------------------------------
    //----------------------------------Helper Functions-------------------------------------------
    //---------------------------------------------------------------------------------------------
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format(
                lift.toString(true) + "\n" +
                arm.toString(true) +  "\n" +
                wrist.toString() + "\n " +
                activeIntake. toString() + "\n" +
                claw.toString() + "\n" +
                parkFlag.toString() + "\n" +
                super.getDriveTrainCurrent() + "\n" +
                "Current Scoring Type: %s\n",
                currentState
        );
    }
}
