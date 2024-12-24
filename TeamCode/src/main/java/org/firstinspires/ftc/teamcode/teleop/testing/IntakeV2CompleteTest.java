package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.version2.ActiveIntakeV2;
import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Intake Complete Test", group = "Subsystem Tests")
public class IntakeV2CompleteTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeArm arm = new IntakeArm(hardwareMap);
        ActiveIntakeV2 intake = new ActiveIntakeV2(hardwareMap);

        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        waitForStart();
        arm.init();
        while(opModeIsActive()){

            if(controller1.a.onPress()){
                intake.toggleBackDoor();
            }
            intake.setPower(controller1.left_stick_y);


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
            telemetry.addLine();
            telemetry.addLine("Gamepad1 A to toggle ejector");
            telemetry.addLine("Gamepad1 Left Stick Y to power Intake Motor");
            telemetry.addLine(intake.toString());
            telemetry.update();
            intake.updateSensor();
            controller1.update();
        }
    }
}
