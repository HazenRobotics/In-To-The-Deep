package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.version2.CraneArm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

//@TeleOp(name = "Crane Test", group="Subsystem Tests")
public class CraneTest extends LinearOpMode {

    ServoImplEx forward, backward;

    @Override
    public void runOpMode() throws InterruptedException {
        CraneArm craneArm = new CraneArm(hardwareMap);
        GamepadEvents controller1 = new GamepadEvents(gamepad1);

//        forward = hardwareMap.get(ServoImplEx.class,"forward");
//        backward = hardwareMap.get(ServoImplEx.class,"backward");
//        forward.setDirection(Servo.Direction.REVERSE);
//
//        forward.setPwmRange(new PwmControl.PwmRange(500,2500));
//        backward.setPwmRange(new PwmControl.PwmRange(500,2500));
//
//        double position=0.5, rotation=0;

        waitForStart();
        craneArm.init();
        while (opModeIsActive()) {
//            craneArm.setOffset(-gamepad1.left_stick_y);
//            craneArm.setPosition(-gamepad1.right_stick_y);
//            if(gamepad1.a) {
//                craneArm.swap();
//            } else {
//                craneArm.swap1();
//            }


            if(controller1.y.onPress()){
                craneArm.goToPosition(CraneArm.CraneStates.TRANSFER);
            }
            if(controller1.b.onPress()){
                craneArm.goToPosition(CraneArm.CraneStates.SPECIMEN_INTAKE);
            }
            if (controller1.a.onPress()){
                craneArm.goToPosition(CraneArm.CraneStates.SPECIMEN_DEPOSIT);
            }
            if(controller1.x.onPress()){
                craneArm.goToPosition(CraneArm.CraneStates.SAMPLE_DEPOSIT);
            }

            if(controller1.left_bumper.onPress()){
                craneArm.openClaw();
            }
            if(controller1.right_bumper.onPress()){
                craneArm.closeClaw();
            }

            craneArm.adjustPositionRotation(controller1.left_stick_y, controller1.right_stick_y);




//            position = MiscMethods.clamp(position + controller1.left_stick_y * 0.002, 0.1, 0.85);
//            rotation = MiscMethods.clamp(rotation + controller1.right_stick_y * 0.002, -0.1, 0.1);
//
//            forward.setPosition(  position + rotation  +  gamepad2.left_stick_y);
//            backward.setPosition( position - rotation + gamepad2.right_stick_y);
//
//            controller1.update();
//            telemetry.addData("Forward Pos: ", forward.getPosition());
//            telemetry.addData("Backward Pos: ", backward.getPosition());
//            telemetry.addData("Position: ", position);
//            telemetry.addData("Rotation: ", rotation);
//            telemetry.addData("Difference: ", forward.getPosition() - backward.getPosition());

            telemetry.addLine(craneArm.toString());
            telemetry.update();
            controller1.update();
        }


    }
}
