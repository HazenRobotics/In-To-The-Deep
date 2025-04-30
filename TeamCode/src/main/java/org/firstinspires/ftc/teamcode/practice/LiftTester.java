package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="LiftTester")
public class LiftTester extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        GamepadEvents gamepad = new GamepadEvents(gamepad2);
        LiftPractice lift = new LiftPractice(hardwareMap, "liftLeft", "liftRight");
        waitForStart();
        while (opModeIsActive()){
            lift.adjustLiftMotor(gamepad.left_trigger.getTriggerValue()-gamepad.right_trigger.getTriggerValue());
            gamepad.update();

        }

    }

}
