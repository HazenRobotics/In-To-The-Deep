package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "IntakePractice")
public class TestIntakePractice extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakePractice intake = new IntakePractice(hardwareMap, "intake");
        GamepadEvents gamepad = new GamepadEvents(gamepad1);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad.a.onPress()){
                intake.powerOn();
            }
            if (gamepad.b.onPress()){
                intake.powerOff();
            }
            gamepad.update();

        }
    }

}
