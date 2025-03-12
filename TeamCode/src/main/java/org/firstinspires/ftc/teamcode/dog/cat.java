package org.firstinspires.ftc.teamcode.dog;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name= "bug")
public class cat extends LinearOpMode {
    GamepadEvents xbox = new GamepadEvents(gamepad1);
    DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
    DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
    DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
    DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
