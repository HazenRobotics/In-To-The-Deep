package org.firstinspires.ftc.teamcode.teleop.version2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.Version2;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;


@TeleOp (name="V2 TeleOp")
public class Version2TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Version2 robot = new Version2(hardwareMap);
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        GamepadEvents controller2 = new GamepadEvents(gamepad2);

        waitForStart();
        robot.completeInit();
        while(opModeIsActive()){
            double forward = controller1.left_stick_y;
            double strafe = controller1.left_stick_x;
            double rotate = controller1.right_stick_x;
            robot.drive(forward,strafe,rotate);

            if(controller1.y.onPress()){
                robot.transferPosition();
            }
            if(controller1.b.onPress()){
                robot.specimenIntake();
            }
            if(controller1.a.onPress()){
                robot.specimenDeposit();
            }
            if(controller1.x.onPress()){
                robot.sampleDeposit();
            }
            if(controller1.right_bumper.onPress()){
                robot.toggleIntake();
            }
            if(controller1.left_bumper.onPress()){
                robot.dropIntake();
            }
            if (controller1.dpad_up.onPress()){
                robot.ejectUp();
            }
            if (controller1.dpad_down.onPress()){
                robot.ejectDown();
            }
            robot.triggerControls(controller1.right_trigger.getTriggerValue() - controller1.left_trigger.getTriggerValue());

            //Driver 2 Manual Controls
            if (controller2.y.onPress()){
                if (robot.isIntakeActive()){
                    robot.deactivateIntake();
                }
                else{
                    robot.activateIntake();
                }
            }
            if (controller2.x.onPress()){
                if (robot.isIntakeActive()){
                    robot.deactivateIntake();
                }
                else{
                    robot.reverseIntake();
                }
            }
            if(controller2.b.onPress()){
                robot.openClaw();
            }
            if(controller2.a.onPress()){
                robot.closeClaw();
            }

            if(controller2.left_bumper.onPress()){
                robot.toggleSweeper();
            }
            if(controller2.right_bumper.onPress()){
                robot.ejectToggle();
            }

            robot.setIntakeArmPos(-controller2.left_stick_y);
            robot.setIntakeWristPos(controller2.left_stick_x);
            robot.setDepositArmPos(-controller2.right_stick_y);
            robot.setDepositWristPos(controller2.right_stick_x);

            controller1.update();
            controller2.update();
            telemetry.addLine(robot.toString());
            telemetry.update();
            robot.updatePID();
        }
    }
}
