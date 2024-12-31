package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.version2.CraneArm;
@TeleOp(name = "")
public class CraneTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CraneArm craneArm = new CraneArm(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            craneArm.setOffset(-gamepad1.left_stick_y);
            craneArm.setPosition(-gamepad1.right_stick_y);
            if(gamepad1.a) {
                craneArm.swap();
            } else {
                craneArm.swap1();
            }
        }


    }
}
