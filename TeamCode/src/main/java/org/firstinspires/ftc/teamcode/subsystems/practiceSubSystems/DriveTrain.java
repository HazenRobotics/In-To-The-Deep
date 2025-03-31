package org.firstinspires.ftc.teamcode.subsystems.practiceSubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    DcMotorEx fl, fr, bl, br;
    /**
     @param: HardwareMap hardwareMap
     @param: String frontLeftMotor
     @param: String frontRightMotor
     @param: String backLeftMotor
     @param: String backRightMotor
     */
    public DriveTrain(HardwareMap hardwareMap, String fl, String fr, String bl, String br)
    {
        this.fl = hardwareMap.get(DcMotorEx.class, fl);
        this.fr = hardwareMap.get(DcMotorEx.class, fr);
        this.bl = hardwareMap.get(DcMotorEx.class, bl);
        this.br = hardwareMap.get(DcMotorEx.class, br);
    }

    /**
     @param: double forward
     @param: double strafe
     @param: double rotate
     */
    public void drive(double forward, double strafe, double rotate)
    {
        fl.setPower(forward + strafe + rotate);
        bl.setPower(forward - strafe + rotate);
        fr.setPower(forward - strafe - rotate);
        br.setPower(forward + strafe - rotate);
    }
}
