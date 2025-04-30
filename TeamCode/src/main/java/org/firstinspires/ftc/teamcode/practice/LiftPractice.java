package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPractice {
    DcMotorEx liftMotorRight, liftMotorLeft;

    public LiftPractice(HardwareMap hardwareMap, String liftLeftMotor, String liftRightMotor)

    {
        liftMotorLeft = hardwareMap.get(DcMotorEx.class, liftLeftMotor);
        liftMotorRight = hardwareMap.get(DcMotorEx.class, liftRightMotor);
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void adjustLiftMotor(double increment){
        liftMotorLeft.setPower(increment);
        liftMotorRight.setPower(increment);
    }

}
