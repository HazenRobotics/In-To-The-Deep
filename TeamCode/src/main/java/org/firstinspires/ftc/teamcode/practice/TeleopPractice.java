package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "Lincoln Teleop")
public class TeleopPractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmPractice arm = new ArmPractice(hardwareMap, "armLeft","armRight");
        GamepadEvents gamepad = new GamepadEvents(gamepad1);
        GamepadEvents controller2 = new GamepadEvents(gamepad2);

        IntakePractice intake = new IntakePractice(hardwareMap, "intake");
        DrivePractice drive = new DrivePractice(hardwareMap, "FLM", "BLM", "FRM", "BRM");
        PivotPractice pivot = new PivotPractice(hardwareMap, "wrist");
        LiftPractice lift = new LiftPractice(hardwareMap, "liftLeft", "liftRight");
        waitForStart();
        while (opModeIsActive())
        {

            pivot.adjustPivotServo(controller2.right_stick_y);
            telemetry.addData("position", pivot.getPosition());
             drive.moveBot(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
            if (gamepad.a.onPress()) {
                intake.powerOn();
            }
            if (gamepad.b.onPress()){
                intake.powerOff();
            }
            if (gamepad.x.onPress()){
                intake.powerReverse();
            }
            arm.moveArm(gamepad.left_trigger.getTriggerValue()-gamepad.right_trigger.getTriggerValue());

            lift.adjustLiftMotor(controller2.left_trigger.getTriggerValue()-gamepad.right_trigger.getTriggerValue());
            gamepad.update();
            controller2.update();
            telemetry.update();
        }
    }
}