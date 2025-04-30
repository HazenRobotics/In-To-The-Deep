package org.firstinspires.ftc.teamcode.practice;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.versionpepto.Arm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="PivotTester")
public class PivotTester extends LinearOpMode {

    public void runOpMode() throws  InterruptedException {
        GamepadEvents gamepad = new GamepadEvents(gamepad1);
        PivotPractice pivot = new PivotPractice(hardwareMap, "wrist");
        waitForStart();
        while (opModeIsActive()){
            pivot.adjustPivotServo(gamepad.right_stick_y);
            telemetry.addData("position", pivot.getPosition());
            telemetry.update();
            gamepad.update();
        }
    }
}
