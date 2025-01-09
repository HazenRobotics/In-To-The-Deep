package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class ExtendoV2ManualTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        Encoder encoderExt = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BRM")));

        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        Encoder encoderLift = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FLM")));

        waitForStart();
        while (opModeIsActive()){

            extendo.setPower(gamepad1.left_stick_y);

            liftLeft.setPower(gamepad1.right_stick_y);
            liftRight.setPower(gamepad1.right_stick_y);

            telemetry.addData("Extendo Power: ", extendo.getPower());
            telemetry.addData("Extendo", encoderExt.getPositionAndVelocity().position);
            telemetry.addData("Extendo Current", extendo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lift Power: ",liftLeft.getPower());
            telemetry.addData("Lift Power: ",liftRight.getPower());
            telemetry.addData("Lift Position", encoderLift.getPositionAndVelocity().position);
            telemetry.addData("Lift Left Current", liftLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lift Right Current", liftRight.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
