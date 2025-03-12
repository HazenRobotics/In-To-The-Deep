package org.firstinspires.ftc.teamcode.Something;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="PracticeTeleop")
public class Something extends LinearOpMode {
    GamepadEvents Controller = new GamepadEvents(gamepad1);
    DcMotorEx FrontLeftMotor = hardwareMap.get(DcMotorEx.class,"FrontLeftMotor");
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
