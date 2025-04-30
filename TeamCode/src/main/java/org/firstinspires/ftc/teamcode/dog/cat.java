package org.firstinspires.ftc.teamcode.dog;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name= "bug")
public class cat extends LinearOpMode {
    GamepadEvents controller1;
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        controller1 = new GamepadEvents(gamepad1);
        waitForStart();
        while (opModeIsActive()){
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            drive(forward, strafe, rotate);
        }

    }

    public void drive(double forward, double strafe, double rotate)
    {
        strafe *= -1;
        rotate *= -1;
        forward *= 1;
        frontLeft.setPower((forward + strafe + rotate));
        backLeft.setPower((forward - strafe + rotate));
        frontRight.setPower((forward - strafe - rotate));
        backRight.setPower((forward + strafe - rotate));

        controller1.update();
    }

}
