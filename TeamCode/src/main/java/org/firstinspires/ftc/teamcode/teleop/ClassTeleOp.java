package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ClassTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left =  hardwareMap.get(DcMotor.class,"left");
        DcMotor right =  hardwareMap.get(DcMotor.class,"right");
        waitForStart();
        while (opModeIsActive()) {
            left.setPower(-gamepad1.left_stick_y+gamepad1.left_stick_x);
            left.setPower(-(-gamepad1.left_stick_y-gamepad1.left_stick_x));
        }

    }
}
