package org.firstinspires.ftc.teamcode.subsystems.practiceSubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActiveIntake {
    DcMotorEx intake;
    double

    public ActiveIntake(HardwareMap hardwareMap, String intake) {
        this.intake = hardwareMap.get(DcMotorEx.class, intake);
    }

    public void setPower(double power)
    {
        intake.setPower(power);
    }

    public double getPower(double power)
    {
        return intake.getPower();
    }


}
