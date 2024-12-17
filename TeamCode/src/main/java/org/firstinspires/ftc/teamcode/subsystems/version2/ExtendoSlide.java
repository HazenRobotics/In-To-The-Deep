package org.firstinspires.ftc.teamcode.subsystems.version2;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;


public class ExtendoSlide extends PIDController {
    DcMotorEx extendo;

    public ExtendoSlide(double Kp, double Ki, double Kd, double Kf) {
        super(Kp, Ki, Kd, Kf);
    }
}
