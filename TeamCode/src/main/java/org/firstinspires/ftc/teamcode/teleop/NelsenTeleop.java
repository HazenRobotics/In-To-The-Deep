package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "NelsenTeleOp")
public class NelsenTeleop extends LinearOpMode {
    final double REST_POWER = 0.21;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        boolean climb = false;
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        double lastPower = 0;
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b) {
                intake.setPower(1);
            } else if (gamepad1.a) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.x) {
                climb=false;
                arm.setPosition(1);
            } else {
                arm.setPosition(0.66);
            }
            if(gamepad1.y) {
                climb=true;
            }
            if(climb) {
                lift.setPower(-1);
            }

            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("LiftPower", lift.getPower());
            telemetry.update();
            lift.setPower(((gamepad1.right_trigger - gamepad1.left_trigger) / 1.5));
            right.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            left.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
            if (lift.getPower() == 0) {
                if (lastPower > 0) {
                    lift.setPower(REST_POWER);
                } else if (lastPower < 0) {
                    lift.setPower(-REST_POWER);

                }
            } else {
                lastPower = lift.getPower();
            }
        }


    }
}
