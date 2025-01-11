package org.firstinspires.ftc.teamcode.teleop.version2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrains.Version2;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArm;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositLift;
import org.firstinspires.ftc.teamcode.subsystems.version2.ExtendoSlide;
import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;


@TeleOp (name="V2 TeleOp")
public class Version2TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Version2 robot = new Version2(hardwareMap);
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
        GamepadEvents controller2 = new GamepadEvents(gamepad2);

        controller2.left_trigger.setPressTolerance(0.5);
        controller2.right_trigger.setPressTolerance(0.5);
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
                robot.toggleSweeper();
            }
            if(controller2.a.onPress()){
                robot.ejectToggle();
            }

//            robot.setIntakeArmPos(controller2.left_trigger.getTriggerValue() - controller2.right_trigger.getTriggerValue());
            if (controller2.left_trigger.onPress()){
                robot.setIntakeArmPos(5);
            }
            if (controller2.right_trigger.onPress()){
                robot.setIntakeArmPos(-5);
            }

            robot.setIntakeWristPos(-controller2.right_stick_y);
            robot.setDepositArmPos(controller2.left_stick_y);

            if(controller2.left_bumper.onPress()){
                robot.setDepositWristPos(5);
            }
            if(controller2.right_bumper.onPress()){
                robot.setDepositWristPos(-5);
            }

            if(controller2.dpad_up.onPress()){
                robot.inverseTriggerControls(0.2);
            }
            if(controller2.dpad_down.onPress()){
                robot.inverseTriggerControls(-0.2);
            }
            if (controller2.dpad_right.onPress() && controller2.a.getValue()){
                robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND);
                robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_SIDEWAYS);
                robot.arm.goToPosition(IntakeArm.IntakeArmStates.HOVER);
                robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_INTAKE_SIDEWAYS);
                robot.closeClaw();
                robot.ejectDown();
            }
            if(controller2.dpad_left.onPress()){
                robot.deposit.toggleClaw();
            }

//            robot.setIntakeArmPos(-controller2.left_stick_y);
//            robot.setIntakeWristPos(controller2.left_stick_x);
//            robot.setDepositArmPos(-controller2.right_stick_y);
//            robot.setDepositWristPos(controller2.right_stick_x);

            controller1.update();
            controller2.update();
            telemetry.addLine(robot.toString());
            telemetry.update();
            robot.updatePID();
        }
    }
}
