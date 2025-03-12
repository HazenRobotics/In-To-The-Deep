package org.firstinspires.ftc.teamcode.pr;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "A Pepto Challenge 3")
public class PeptoChallenge3 extends LinearOpMode {
    GamepadEvents controller1;
    public final double MAX_POWER = 1;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    @Override
    public void runOpMode() throws InterruptedException {

        controller1 = new GamepadEvents(gamepad1);

        //You can change these controls and see what happens!
        backLeft = hardwareMap.get(DcMotorEx.class,"FLM");
        frontLeft = hardwareMap.get(DcMotorEx.class,"BLM");
        backRight = hardwareMap.get(DcMotorEx.class,"FRM");
        frontRight = hardwareMap.get(DcMotorEx.class,"BRM");

        //Changes direction of certain motors
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection((DcMotorSimple.Direction.FORWARD));

        waitForStart();
        while(opModeIsActive())
        {
            drive(controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);

            controller1.update();

        }

    }
    //This controls the driving of the robot
    public void drive(double forward, double strafe, double rotate)
    {
        //Invert strafe and rotate so that all positive values can be fed into the function to drive
        strafe *= -1;
        rotate *= -0.5;

        frontLeft.setPower((forward + strafe + rotate)*MAX_POWER);
        backLeft.setPower((forward - strafe + rotate)*MAX_POWER);
        frontRight.setPower((forward - strafe - rotate)*MAX_POWER);
        backRight.setPower((forward + strafe - rotate)*MAX_POWER);
    }

}

