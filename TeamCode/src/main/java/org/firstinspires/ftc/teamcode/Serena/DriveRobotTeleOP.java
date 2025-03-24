package org.firstinspires.ftc.teamcode.Serena;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="A test Drive")
public class DriveRobotTeleOP extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException
    {
        GamepadEvents controller = new GamepadEvents(gamepad1);
        DriveRobot driver = new DriveRobot(hardwareMap, "frontLeft", "bottomLeft", "frontRight", "bottomRight");

        waitForStart();
        while(opModeIsActive())
        {
            driver.drive(-controller.left_stick_y, controller.left_stick_x, controller.right_stick_x);
            controller.update();
        }
    }
}
