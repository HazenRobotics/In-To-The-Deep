package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "HandPractice")
public class ArmTesterPractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmPractice arm = new ArmPractice(hardwareMap, "armLeft","armRight");
        GamepadEvents gamepad = new GamepadEvents(gamepad1);
        waitForStart();
        while (opModeIsActive())
        {
            arm.moveArm(gamepad.left_trigger.getTriggerValue()-gamepad.right_trigger.getTriggerValue());
            gamepad.update();
        }
    }
}
