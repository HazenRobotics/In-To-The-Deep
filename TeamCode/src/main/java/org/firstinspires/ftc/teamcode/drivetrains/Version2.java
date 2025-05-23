package org.firstinspires.ftc.teamcode.drivetrains;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.version2.ActiveIntakeV2;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArm;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArmV2;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositLift;
import org.firstinspires.ftc.teamcode.subsystems.version2.ExtendoSlide;
import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
import org.firstinspires.ftc.teamcode.subsystems.version2.Sweeper;
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensor;

public class Version2 extends Mecanum{

    public ActiveIntakeV2 intake;
    public IntakeArm arm;
    public ExtendoSlide extendo;
    public DepositLift lift;
    public Sweeper sweeper;
    public DepositArmV2 deposit;

    boolean delayEjector, delayArm, delaySpecDeposit, delayTransfer;

    boolean autoPIDAcitve = true;

    public Version2(HardwareMap hw){
        super(hw);
        intake = new ActiveIntakeV2(hw);
        arm = new IntakeArm(hw);
        extendo = new ExtendoSlide(hw);
        lift = new DepositLift(hw);
        deposit = new DepositArmV2(hw);
        sweeper = new Sweeper(hw);
        delayEjector = false;
        delayArm = false;
    }

    public void init(){
        arm.init();
        deposit.init();
        sweeper.init();
    }
    public void completeInit(){
        init();
        extendo.resetExtendoOffset();
        lift.resetLiftOffset();
    }
    public void specAutoInit(){
        arm.init();
        sweeper.init();
        extendo.resetExtendoOffset();
        lift.resetLiftOffset();
    }

    public void transferPosition(){
        deposit.openClaw();
        extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
        lift.goToPosition(DepositLift.LiftStates.TRANSFER);
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
//        deposit.goToPosition(DepositArmV2.PivotArmStates.TRASNFER);
        deactivateIntake();
        intake.dropDown();
        delayTransfer = true;
    }

    public void specimenIntake(){
        lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE);
//        deposit.goToPosition(DepositArmV2.PivotArmStates.SPECIMEN_INTAKE);
        deposit.openClaw();
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
        extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
        if (intake.isEjectorDown()){
            delayEjector = true;
        }else{
            intake.dropDown();
        }
    }
    public void sampleIntake(){
        lift.goToPosition(DepositLift.LiftStates.TRANSFER);
        deposit.goToPosition(DepositArmV2.PivotArmStates.TRASNFER);
        deposit.openClaw();
        arm.goToPosition(IntakeArm.IntakeArmStates.INTAKE);
        extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND);
        activateIntake();
    }
    public void specimenDeposit(){
        lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT);
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
        deposit.goToPosition(DepositArmV2.PivotArmStates.SPECIMEN_DEPOSIT);
        deposit.closeClaw();
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
        extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
//        delaySpecDeposit = true;
    }

    public void sampleDeposit(){
        lift.goToPosition(DepositLift.LiftStates.SAMPLE_DEPOSIT);
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
//        deposit.goToPosition(DepositArmV2.PivotArmStates.SAMPLE_DEPOSIT);
        deposit.closeClaw();
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
        extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
        delayArm = true;
    }

    /**
     * Depending on if the lift is in Transfer Mode or not, this (powered by triggers)
     * will extend the horizontal extendo OR the vertical lift.
     * @param power "Speed" of which the driver can use to control the extendo/lift
     */
    public void triggerControls(double power){
        if (Math.abs(power) < 0.01){
            return;
        }
        if(lift.getCurrentState() == DepositLift.LiftStates.TRANSFER){
            extendo.setPosition(power);
        }else{
            lift.setPosition(power);
        }
    }

    /**
     * Likely to be controlled using Right Bumper
     * If IntakeArm is in transfer position, then this implies that sample intake is NOT active/desired
     * If arm is in transfer mode, then only claw will be activated
     */
    public void toggleIntake(){
        //Lift IS down (intake mode)
        if (arm.getCurrentState() == IntakeArm.IntakeArmStates.TRANSFER){
            deposit.toggleClaw();
            deactivateIntake();//Deactivates intake if it was still running for whatever reason
        }
        //Lift is NOT down (deposit mode)
        else{
            if (intake.getPower() != 0){
                deactivateIntake();
            }
            else if(arm.getCurrentState() == IntakeArm.IntakeArmStates.INTAKE){
                activateIntake();
            }
            else{
                reverseIntake();
            }
            //Else probably either eject via ejection servo or reverse intake.
        }
    }

    /**
     * Likely to be controlled using Left Bumper
     * This toggles the intakeArm to drop down (below submersible bar) or to stay in stow/transfer position
     */
    public void dropIntake(){
        if (extendo.getCurrentState() == ExtendoSlide.ExtendoStates.VARIABLE_EXTEND){
            if(arm.getCurrentState() == IntakeArm.IntakeArmStates.INTAKE){
                arm.goToPosition(IntakeArm.IntakeArmStates.HOVER);
                deactivateIntake();
            }else{
                arm.goToPosition(IntakeArm.IntakeArmStates.INTAKE);
//                activateIntake();
            }
        }
        else if (extendo.getCurrentState() == ExtendoSlide.ExtendoStates.TRANSFER){
            if(arm.getCurrentState() == IntakeArm.IntakeArmStates.HOVER){
                arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
                deactivateIntake();
            }else{
                arm.goToPosition(IntakeArm.IntakeArmStates.HOVER);
            }
        }
    }


    public void updatePID(){
        extendo.updatePID();
        lift.updatePID();
        intake.updateSensor();
        if (lift.getPosition() > 100 + DepositLift.LiftStates.TRANSFER.getPosition()){
            if(delayArm && Math.abs(extendo.getVelocity()) < 20 ){
                deposit.goToPosition(DepositArmV2.PivotArmStates.SAMPLE_DEPOSIT);
                delayArm = false;
            }
            if(delayEjector){
                ejectUp();
                deposit.goToPosition(DepositArmV2.PivotArmStates.SPECIMEN_INTAKE);
                delayEjector = false;
            }
        }
        if(delaySpecDeposit && (lift.getPosition() + 10) > DepositLift.LiftStates.SPECIMEN_INTAKE.getPosition()){
            delaySpecDeposit = false;
        }

        //This implies a variable extend state
        if (lift.getCurrentState() == DepositLift.LiftStates.TRANSFER && extendo.getPosition() > 50 + extendo.getExtendoOffset()){
            deposit.goToPosition(DepositArmV2.PivotArmStates.TEMP_TRANSFER);
            openClaw();
        }
        //This implies a transfer state
        else if(lift.getCurrentState() == DepositLift.LiftStates.TRANSFER && extendo.getPosition() < 50 + extendo.getExtendoOffset()){
            deposit.goToPosition(DepositArmV2.PivotArmStates.TRASNFER);
        }
    }
    //----------------------------------------------------------------------------------------------
    //-----------------------------------Auto Actions-----------------------------------------------
    //----------------------------------------------------------------------------------------------
    public Action autoPID(){
        autoPIDAcitve = true;
        return new AutoPID();
    }
    public void disableAutoPID(){
        autoPIDAcitve = false;
    }

    public Action waitForLift(double waitSeconds){
        return new WaitForLift(waitSeconds);
    }
    public Action waitForLift(double waitSeconds, int posError, int velError){
        return new WaitForLift(waitSeconds, posError, velError);
    }

    public Action waitForExtendo(long waitSeconds){
        return new WaitForExtendo(waitSeconds);
    }
    public Action waitForExtendo(long waitSeconds, int posError, int velError){
        return new WaitForExtendo(waitSeconds, posError, velError);
    }
    public Action waitForExtendo(long waitSeconds, int posError, int velError, boolean useColor){
        return new WaitForExtendo(waitSeconds, posError, velError, useColor);
    }
    public Action waitForExtendo(long waitSeconds, boolean useColor){
        return new WaitForExtendo(waitSeconds, 30, 10, useColor);
    }
    class AutoPID implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            updatePID();
            return autoPIDAcitve;
        }
    }

    class WaitForLift implements Action{

        long endTime;
        int positionError, velocityError;

        boolean init;
        public WaitForLift(double waitSeconds){
            this(waitSeconds, 50, 20);
        }
        public WaitForLift(double waitSeconds, int posError, int velError){
            endTime =(long) waitSeconds * 1000;
            positionError = posError;
            velocityError = velError;
            this.init = true;
        }

        /**
         * Returns true if this is is supposed to loop again, returns false to stop
         * @param telemetryPacket
         * @return
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (init){
                this.endTime += System.currentTimeMillis();
                init = false;
                return true;
            }
            return (System.currentTimeMillis() < endTime) &&
                    (Math.abs(lift.getTarget() - lift.getPosition()) > positionError ||
                            lift.getVelocity() > velocityError);
        }
    }
    class WaitForExtendo implements Action{

        long endTime;
        int positionError, velocityError;
        boolean init, useColor;
        public WaitForExtendo(long waitSeconds){
            this(waitSeconds, 30, 10);
        }
        public WaitForExtendo(long waitSeconds, int posError, int velError){
            this.endTime = waitSeconds * 1000;
            this.positionError = posError;
            this.velocityError = velError;
            this.init = true;
        }

        public WaitForExtendo(long waitSeconds, int posError, int velError, boolean useColor){
            this(waitSeconds,posError,velError);
            this.useColor = useColor;
        }

        /**
         * Returns true if this is is supposed to loop again, returns false to stop
         * @param telemetryPacket
         * @return
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //If color sensor is required, this will automatically end the wait when the color sensor detects something
            if (init){
                this.endTime += System.currentTimeMillis();
                init = false;
                return true;
            }
            if (useColor){
                return (System.currentTimeMillis() < this.endTime) &&
                        (
                                Math.abs(extendo.getTarget() - extendo.getPosition()) > this.positionError ||
                                        Math.abs(extendo.getVelocity()) > this.velocityError
                        )
                        &&
                        intake.checkIntakeState() == ColorSensor.Color.None;
            }
            //Otherwise, default to the standard time/position checks
            return (System.currentTimeMillis() < this.endTime) &&
                    (
                            Math.abs(extendo.getTarget() - extendo.getPosition()) > this.positionError ||
                            Math.abs(extendo.getVelocity()) > this.velocityError
                    );
        }
    }

    //----------------------------------------------------------------------------------------------
    //-----------------------------------Manual Controls--------------------------------------------
    //----------------------------------------------------------------------------------------------

    //Intake Controls
    public void activateIntake(){
        intake.setPower(1);
    }
    public void deactivateIntake(){
        intake.setPower(0);
    }
    public void reverseIntake(){
        intake.setPower(-1);
    }
    public boolean isIntakeActive(){
        return intake.getPower() != 0;
    }
    public void ejectUp(){
        intake.bringUp();
    }
    public void ejectDown(){
        intake.dropDown();
    }
    public void ejectToggle(){
        intake.toggleBackDoor();
    }

    //Arm Controls
    public void setIntakeArmPos(double power){
        arm.adjustPositionArm(power);
    }
    public void setIntakeWristPos(double power){
        arm.adjustPositionWrist(power);
    }

    //Deposit Arm Controls
    public void setDepositArmPos(double power){
        deposit.adjustPositionArm(power);
    }

    public void setDepositWristPos(double power){
        deposit.adjustPositionWrist(power);
    }
    //Claw Controls
    public void openClaw(){
        deposit.openClaw();
    }
    public void closeClaw(){
        deposit.closeClaw();
    }

    public void inverseTriggerControls(double power){
        if (Math.abs(power) < 0.01){
            return;
        }
        if(lift.getCurrentState() == DepositLift.LiftStates.TRANSFER){
            lift.setPositionInverse(power);
        }
        //One of these should always be true, but this is just an extra safe guard
        else {
            extendo.setPositionInverse(power);
        }
    }
    public void sweeperSweeper(boolean sweep){
        if (sweep){
            sweeper.sweepSweeper();
        }else{
            sweeper.stowSweeper();
        }
    }

    public void toggleSweeper(){
        sweeper.toggleSweeper();
    }



    @NonNull
    public String toString(){
        return String.format(
                "Intake: %s\n" +
                "Arm: %s\n" +
                "Extendo: %s\n" +
                "Lift: %s\n" +
                "Deposit: %s\n",
                intake.toString(),
                arm.toString(),
                extendo.toString(),
                lift.toString(),
                deposit.toString());
    }

}
