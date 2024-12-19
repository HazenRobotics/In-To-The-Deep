package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.IndicatorLight;
@TeleOp(name="lighttester")
public class ColorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IndicatorLight indicatorLight = new IndicatorLight(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()) {
            indicatorLight.setColor(gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
