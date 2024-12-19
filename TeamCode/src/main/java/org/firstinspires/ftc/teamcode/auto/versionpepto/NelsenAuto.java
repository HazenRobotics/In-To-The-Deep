package org.firstinspires.ftc.teamcode.auto.versionpepto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@Autonomous(name = "NelesenAuto")
public class NelsenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        lift.setPower(0.5);
        lift.setTargetPosition(1300);
        arm.setPosition(0.66);

        while (lift.getCurrentPosition()<1300 && opModeIsActive()) {
            telemetry.addData("lift", lift.getCurrentPosition());

            telemetry.update();
        }
        lift.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        long time = System.currentTimeMillis();
        while (time+200>System.currentTimeMillis());
        intake.setPower(1);
        time = System.currentTimeMillis();
        while (time+200>System.currentTimeMillis());


        lift.setPower(-1);
        lift.setTargetPosition(100);
        while (lift.getCurrentPosition()>100 && opModeIsActive()) {
            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.update();
        }


    }}
