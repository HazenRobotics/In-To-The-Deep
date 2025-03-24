package org.firstinspires.ftc.teamcode.Serena;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private CRServo activeintake;

    public Intake(HardwareMap hw, String activeintake)
    {
     this.activeintake = hw.get(CRServo.class, activeintake);
    }

    public void activeservo()
    {
        activeintake.setPower(0.7);
    }
    public void deactivatedservo()
    {
        activeintake.setPower(0);
    }
}
