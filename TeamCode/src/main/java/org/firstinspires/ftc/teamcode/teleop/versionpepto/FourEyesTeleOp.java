package org.firstinspires.ftc.teamcode.teleop.versionpepto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drivetrains.FourEyesRobot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.vision.processors.SampleProcessor2;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="Pepto Bismal")
public class FourEyesTeleOp extends LinearOpMode {

    private boolean easterEgg = false;
    FourEyesRobot fourEyesRobot;
    GamepadEvents controller1,controller2;
    public static GamepadEvents.GamepadButton[] binding1;
    public static GamepadEvents.GamepadButton[] binding2;
    Servo light;
    double lightNumber=0.277;
    SampleProcessor2 visionProc;

    @Override
    public void runOpMode() throws InterruptedException {

        fourEyesRobot  = new FourEyesRobot(hardwareMap);
        controller1 = new GamepadEvents(gamepad1);
        controller2 = new GamepadEvents(gamepad2);
        binding1 = new GamepadEvents.GamepadButton[] {
                controller1.left_bumper,
                controller1.right_bumper,
                controller1.y,
                controller1.b,
                controller1.a,
                controller1.x,
                controller1.dpad_up,
                controller1.dpad_down

        };
        binding2 = new GamepadEvents.GamepadButton[] {
                controller2.y,
                controller2.b,
                controller2.a,
                controller2.x,
                controller2.dpad_up,
                controller2.dpad_down,
                controller2.left_bumper,
                controller2.dpad_left,
                controller2.dpad_right
        };
        ElapsedTime timer = new ElapsedTime();

        light = hardwareMap.get(Servo.class,"light");
        visionProc = new SampleProcessor2(telemetry);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get( WebcamName.class, "Webcam 1"))
                .addProcessor(visionProc)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();
        fourEyesRobot.initializePowerStates();
        timer.reset();

        while (opModeIsActive()) {

            fourEyesRobot.drive(controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);

            //User 1 controls, driver automations
            //Left bumper
            if(binding1[0].onPress()) {
               fourEyesRobot.toggleIntake();
            }
            //Right bumper
            if(binding1[1].onPress()) {
                fourEyesRobot.toggleDeposit();
            }
            //Y button
            if(binding1[2].onPress()) {
                fourEyesRobot.intakeSamplePos();
            }
            //B button
            if(binding1[3].onPress()) {
                fourEyesRobot.intakeSpecimenPos();
            }
            //A button
            if(binding1[4].onPress()) {
                fourEyesRobot.depositSpecimenPos();
            }
            //X button
            if(binding1[5].onPress()) {
                fourEyesRobot.depositSamplePos();
            }
            //Dpad Up
            if(binding1[6].onPress() || binding2[4].onPress()){
                fourEyesRobot.raiseClimb();
            }
            //Dpad Down
            if(binding1[7].onPress() || binding2[5].onPress()){
                fourEyesRobot.lowerClimb();
            }
            if (easterEgg){
                if(binding2[6].onPress()){
                    fourEyesRobot.toggleFlagWave();
                }
                //3 minutes since start phase
                else if (timer.seconds() > 180) {
                    fourEyesRobot.turnOnWave();
                }
            }
            if (binding2[7].onPress()){
                fourEyesRobot.raiseFlag();
            }
            if(binding2[8].onPress()){
                fourEyesRobot.stowFlag();
            }

            if (controller2.left_trigger.getTriggerValue() > 0.9 && controller2.right_trigger.getTriggerValue() > 0.9){
                fourEyesRobot.resetLift();
                fourEyesRobot.resetArmStartOffset();
                gamepad2.rumble(1000);
            }

            //End game warning
            if (timer.seconds() > 90){
                gamepad1.rumble(1000);
            }
            if (90 < timer.seconds() && timer.seconds() < 95) {
                gamepad1.stopRumble();
            }

            //Player 2 controls
            //Manual driving
            //Lift manual control
            fourEyesRobot.moveLift(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());
            //Arm manual control
            fourEyesRobot.changeHeightArm(controller2.right_stick_y);
            //Wrist manual control
            fourEyesRobot.setWristPosition(-controller2.left_stick_y);

            //Active intake manual control
            //Y Key
            if(binding2[0].onPress()){
                if (fourEyesRobot.isIntaking()){
                    fourEyesRobot.deactivateIntake();
                }
                else {
                    fourEyesRobot.intakeBackward();
                }
            }
            //X Key
            if(binding2[3].onPress()){
                if (fourEyesRobot.isIntaking()){
                    fourEyesRobot.deactivateIntake();
                }
                else{
                    fourEyesRobot.activateIntake();
                }
            }
            //Claw manual control
            //B Key
            if(binding2[1].onPress()){
                fourEyesRobot.openClaw();
            }
            //A Key
            if(binding2[2].onPress()){
                fourEyesRobot.closeClaw();
            }
//            //Dpad Up
//            if(binding2[4].onHeldFor(2000)){
//                fourEyesRobot.resetIMU();
//            }
//            //Dpad Down
//            if(binding2[5].onHeldFor(1000)){
//                fourEyesRobot.toggleFieldCentric();
//            }\
            if(fourEyesRobot.activeIntake.getSampleCaptured()) {
                switch (visionProc.getIntakeStatus()) {
                    case RED:
                        light.setPosition(0.277);
                    case BLUE:
                        light.setPosition(0.611);
                    case YELLOW:
                        light.setPosition(0.388);
                    case NOTHING:
                        light.setPosition(0.5);
                }
            } else if(timer.seconds()>110) {
                light.setPosition(lightNumber);
                lightNumber+=0.0005;
                if(lightNumber>0.722) {
                    lightNumber=0.277;
                }

            } else {
                light.setPosition(0);
            }

            fourEyesRobot.updatePID();
            controller1.update();
            controller2.update();
            telemetry.addLine(fourEyesRobot.toString());
            telemetry.update();

        }

    }
}
