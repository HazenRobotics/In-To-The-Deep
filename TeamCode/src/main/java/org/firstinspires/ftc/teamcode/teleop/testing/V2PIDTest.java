package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.version2.DepositLift;
import org.firstinspires.ftc.teamcode.subsystems.version2.ExtendoSlide;
import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="V2 Extendo PID Tester",group="Subsystems Test")
public class V2PIDTest extends LinearOpMode {

    ExtendoSlide extendo;
    DepositLift lift;

    private double INCREMENT = 0.01;
    double[] pidValues;
    int index;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean extendoInit = true, liftInit = true;
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        GamepadEvents controller2 = new GamepadEvents(gamepad2);
        while(opModeInInit()){

            if(controller1.a.onPress()){
                extendoInit = !extendoInit;
            }
            if (controller1.b.onPress()){
                liftInit = !liftInit;
            }

            if(controller1.dpad_up.onPress()){
                break;
            }

            telemetry.addData("Extendo Selected [A]:",extendoInit);
            telemetry.addData("Lift Selected [B]:",liftInit);
            telemetry.addLine("Dpad Up to Finalize");
            telemetry.update();
            controller1.update();
        }

        int subNum = 0;
        if(extendoInit){
            telemetry.addLine("Initialized Extendo!");
            extendo = new ExtendoSlide(hardwareMap);
            subNum++;
        }
        if(liftInit){
            telemetry.addLine("Initialized Lift!");
            lift = new DepositLift(hardwareMap);
            subNum++;
        }
        telemetry.update();

        index = 0;
        pidValues = new double[subNum * 4];

        if(extendoInit){
            for(double val : extendo.getPIDValues()){
                pidValues[index] = val;
                index++;
            }
        }
        if(liftInit){
            for(double val : lift.getPIDValues()){
                pidValues[index] = val;
                index++;
            }
        }
        index = 0;

        IntakeArm arm = new IntakeArm(hardwareMap);

        waitForStart();
        arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
        if (extendoInit) {extendo.resetExtendoOffset();}
        while(opModeIsActive()){
            if (extendoInit) {
                telemetry.addLine("Controller 1 for Extendo:");
                telemetry.addLine("Triggers for Manual Control, Bumpers for set positions");
                extendo.setPosition(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());
                if (controller1.left_bumper.onPress()){
                    extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
                }
                if (controller1.right_bumper.onPress()){
                    extendo.goToPosition(ExtendoSlide.ExtendoStates.HALF_EXTEND);
                }
                telemetry.addData("Power: ",extendo.updatePID());
            }
            if (liftInit) {
                telemetry.addLine("Controller 2 for Extendo:");
                telemetry.addLine("Triggers for Manual Control, Bumpers for set positions");
                lift.setPosition(controller2.right_trigger.getTriggerValue() - controller2.left_trigger.getTriggerValue());
                if (controller2.left_bumper.onPress()){
                    lift.goToPosition(DepositLift.LiftStates.TRANSFER);
                }
                if (controller2.right_bumper.onPress()){
                    lift.goToPosition(DepositLift.LiftStates.SAMPLE_DEPOSIT);
                }
                telemetry.addData("Lift Power:", lift.updatePID());
            }
            if(controller1.x.onPress() || controller2.x.onPress()){
                INCREMENT *= 10;
            }else if (controller1.y.onPress() || controller2.y.onPress()){
                INCREMENT /= 10;
            }

            //PID Tuning
            //Controls the value increase/decrease
            if (controller1.dpad_up.onPress() || controller2.dpad_up.onPress()){
                pidValues[index] += INCREMENT;
            }
            else if( controller1.dpad_down.onPress() || controller2.dpad_down.onPress()){
                pidValues[index] = Math.max(0, pidValues[index] - INCREMENT);
            }

            //Controls which value is being edited
            if (controller1.dpad_left.onPress() || controller2.dpad_left.onPress()) {
                updateIndex(false);
            }
            else if(controller1.dpad_right.onPress() || controller2.dpad_right.onPress()){
                updateIndex(true);
            }

            //PID Tuning Information
            telemetry.addLine("PID Tuning Information:");
            telemetry.addLine("X/Y to Increase/Decrease [INCREMENT] amount");
            telemetry.addLine("DpadUp/DpadDown to Increase/Decrease current Selected PID by said [INCREMENT] amount");
            telemetry.addLine("DpadLeft/DpadRight to switch between PID values");
            telemetry.addData("Current [INCREMENT] value: ",INCREMENT);
            telemetry.addLine(buildPIDString());

            if(extendoInit){
                extendo.adjustPID(pidValues[0],pidValues[1],pidValues[2], pidValues[3]);
                telemetry.addLine("Line 1 - Extendo PID");
                telemetry.addLine(extendo.toString());
            }
            if(liftInit){
                int offset = pidValues.length - 4;
                lift.adjustPID(pidValues[offset],pidValues[1 + offset],pidValues[2 + offset], pidValues[3 + offset]);
                telemetry.addLine("Line" + pidValues.length/4 +"Lift PID");
                telemetry.addLine(lift.toString());
            }




            telemetry.update();
            controller1.update();
            controller2.update();
        }
    }
    public void updateIndex(boolean increase){
        if (increase){
            index = (index+1) % pidValues.length;
        }else{
            index = (pidValues.length + index - 1) % pidValues.length;
        }
    }

    public String buildPIDString(){
        StringBuilder result = new StringBuilder("PID data:");
        for(int i=0; i<pidValues.length; i++){
            if (i%4 == 0) {
                result.append("\n");
            }
            if (i==index){
                result.append("[").append(pidValues[i]).append("]");
            }else{
                result.append(" ").append(pidValues[i]).append(" ");
            }

        }
        return result.toString();
    }
}
