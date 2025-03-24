package org.firstinspires.ftc.teamcode.Serena;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveRobot {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    public DriveRobot(HardwareMap hw, String fl, String bl, String fr, String br){

        frontLeft = hw.get(DcMotorEx.class, fl);
        backLeft = hw.get(DcMotorEx.class, bl);
        frontRight = hw.get(DcMotorEx.class, fr);
        backRight = hw.get(DcMotorEx.class, br);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void drive(double forward, double strafe, double rotate)
    {
        frontLeft.setPower(forward + strafe + rotate);
        backLeft.setPower(forward - strafe + rotate);
        frontRight.setPower(forward - strafe - rotate);
        backRight.setPower(forward + strafe - rotate);
    }
}
