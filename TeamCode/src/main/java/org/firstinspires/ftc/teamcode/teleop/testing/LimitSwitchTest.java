package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name= "Limit Switch Test",group="Subsystem Tests")
public class LimitSwitchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        DigitalChannel limitSwitch = hardwareMap.digitalChannel.get("bumpSwitch");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Sensor readout: ", limitSwitch.getState());
            telemetry.update();
        }
    }
}
