//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//@TeleOp(name="Robotics Class Drive Code")
//public class RoboticsClassDrive extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        DcMotor left = hardwareMap.get(DcMotor.class,"left");
//        DcMotor right = hardwareMap.get(DcMotor.class,"right");
//        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
//
//        left.setDirection(DcMotorSimple.Direction.REVERSE);
//        right.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        Servo claw = hardwareMap.get(Servo.class,"claw");
//
//        waitForStart();
//
//        while(opModeIsActive()){
//            double drive = gamepad1.left_stick_y;
//            double rotate = gamepad1.right_stick_x;
//
//            left.setPower(drive + rotate);
//            right.setPower(drive - rotate);
//
//            arm.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//
//            if (gamepad1.left_bumper){
//                claw.setPosition(0);
//            }
//
//            if (gamepad1.right_bumper){
//                claw.setPosition(1);
//            }
//
//
//            telemetry.addData("Claw Position:",claw.getPosition());
//            telemetry.update();
//        }
//
//    }
//}
