package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArm;
import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Deposit Arm V2 Test", group="Subsystem Tests")
public class DepositArmV2Test extends LinearOpMode {

    DepositArm arm;
    Servo left, right, wrist;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean leftPiv = false, rightPiv = false, wristPiv = false, intakeArm = false;
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        while (!opModeIsActive()){
            telemetry.addLine("Please use gamepad1 to initialize intake subsystem");
            if(controller1.a.onPress()){
                leftPiv = !leftPiv;
            }
            if(controller1.b.onPress()){
                rightPiv = !rightPiv;
            }
            if(controller1.x.onPress()){
                wristPiv = !wristPiv;
            }
            if(controller1.y.onPress()){
                intakeArm = !intakeArm;
            }

            if(controller1.dpad_up.onPress()){
                break;
            }

            telemetry.addData("LeftPiv   [a]: ",leftPiv);
            telemetry.addData("RightPiv  [b]: ",rightPiv);
            telemetry.addData("wrist     [x]: ",wristPiv);
            telemetry.addData("DepositArm [y]: ",intakeArm);
            telemetry.addLine("Dpad Up to complete!");
            controller1.update();
            telemetry.update();
        }


        if (intakeArm){
            arm = new DepositArm(hardwareMap);
            arm.setPositionArm(0.5);
            arm.setPositionWrist(0.5);
            telemetry.addLine("Intake Arm Initialized");
        }else{
            if (leftPiv){
                left = hardwareMap.get(Servo.class, "depositForward");
                left.setPosition(0.5);
                telemetry.addLine("Left Pivot Servo Initialized");
            }
            if (rightPiv){
                right = hardwareMap.get(Servo.class, "depositBackward");
                right.setDirection(Servo.Direction.REVERSE);
                right.setPosition(0.5);
                telemetry.addLine("Right Pivot Servo Initialized");
            }
            if (wristPiv){
                wrist = hardwareMap.get(Servo.class, "depositWrist");
                telemetry.addLine("Wrist Servo Initialized");
            }
        }

        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            if(intakeArm){
                if (gamepad1.dpad_up){
                    arm.setPositionArm(arm.getPositionArm() + 0.001);
                }
                if (gamepad1.dpad_down){
                    arm.setPositionArm(arm.getPositionArm() - 0.001);
                }
                arm.setPositionWrist(arm.getPositionWrist() + 0.0005 * (gamepad1.left_trigger - gamepad1.right_trigger));

                telemetry.addLine("Gamepad1 Dpad Up and Down for Arm Pivot");
                telemetry.addLine("Gamepad1 Left and Right Trigger for Wrist Pivot");
                telemetry.addLine(arm.toString());
            }
            else {
                if (leftPiv) {
                    left.setPosition(left.getPosition() + 0.001 * gamepad1.left_stick_y);
                    telemetry.addLine("Gamepad1 Left Stick Y to control Left Pivot");
                    telemetry.addData("Front Piv Position:", left.getPosition());
                }
                if (rightPiv) {
                    right.setPosition(right.getPosition() + 0.001 * gamepad1.right_stick_y);
                    telemetry.addLine("Gamepad1 Right Stick Y to control Right Pivot");
                    telemetry.addData("Back Piv Position:", right.getPosition());
                }
                if (wristPiv) {
                    wrist.setPosition(wrist.getPosition() + 0.001 * (gamepad1.left_trigger - gamepad1.right_stick_y));
                    telemetry.addLine("Gamepad1 Left and Right Triggers to control Wrist");
                    telemetry.addData("Wrist Position:", wrist.getPosition());
                }
            }
            if(leftPiv && rightPiv){
                telemetry.addData("Right Pivot Offset: ",right.getPosition() - left.getPosition());
            }
            controller1.update();
            telemetry.update();
        }
    }
}
