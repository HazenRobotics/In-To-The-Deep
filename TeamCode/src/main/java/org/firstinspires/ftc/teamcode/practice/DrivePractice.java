package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivePractice {
    DcMotor frontLeft, backLeft, frontRight, backRight;

    public DrivePractice(HardwareMap hardwareMap, String leftFront, String leftBack, String rightFront, String rightBack) {
        frontLeft = hardwareMap.get(DcMotorEx.class, leftFront);
        backLeft = hardwareMap.get(DcMotorEx.class, leftBack);
        frontRight = hardwareMap.get(DcMotorEx.class, rightFront);
        backRight = hardwareMap.get(DcMotorEx.class, rightBack);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveBot(double forward, double strafe, double rotate){
        frontLeft.setPower(forward + strafe + rotate);
        backLeft.setPower(forward - strafe + rotate);
        frontRight.setPower(forward - strafe - rotate);
        backRight.setPower(forward + strafe - rotate);
    }
}
