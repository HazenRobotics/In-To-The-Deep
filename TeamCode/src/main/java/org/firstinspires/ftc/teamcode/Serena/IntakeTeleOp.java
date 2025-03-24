package org.firstinspires.ftc.teamcode.Serena;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="IntakeTest")
public class IntakeTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEvents buttonintake = new GamepadEvents(gamepad1);
        Intake intake = new Intake(hardwareMap, "CRservo");
`       waitForStart();
        while (opModeIsActive())
        {
            if (buttonintake.b.onPress())
            {
                intake.activeservo();
            }
        }
    }
}
