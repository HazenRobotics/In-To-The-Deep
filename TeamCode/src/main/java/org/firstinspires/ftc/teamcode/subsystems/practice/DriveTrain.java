package org.firstinspires.ftc.teamcode.subsystems.practice;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private final DcMotorEx fl, bl, fr, br;

    public DriveTrain(HardwareMap hw, String frontLeft, String backLeft, String frontRight, String backRight) {
        fl = hw.get(DcMotorEx.class, frontLeft);
        bl = hw.get(DcMotorEx.class, backLeft);
        fr = hw.get(DcMotorEx.class, frontRight);
        br = hw.get(DcMotorEx.class, backRight);
    }

    public void drive(double forward, double strafe, double rotate)
    {
        fl.setPower(forward + strafe + rotate);
        bl.setPower(forward - strafe + rotate);
        fr.setPower(forward - strafe - rotate);
        br.setPower(forward + strafe - rotate);
    }

    public void cubedDrive(double forward, double strafe, double rotate)
    {
        fl.setPower(Math.pow(forward + strafe + rotate, 3));
        bl.setPower(Math.pow(forward - strafe + rotate, 3));
        fr.setPower(Math.pow(forward - strafe - rotate, 3));
        br.setPower(Math.pow(forward + strafe - rotate, 3));
    }
}
