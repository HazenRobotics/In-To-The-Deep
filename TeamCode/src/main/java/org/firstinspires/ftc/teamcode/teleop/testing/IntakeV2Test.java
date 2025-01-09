package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.IndicatorLight;
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensor;

//@TeleOp(name="Intake V2 Test", group="Subsystem Tests")
public class IntakeV2Test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ColorSensor colorSensor = new ColorSensor(hardwareMap);
        IndicatorLight led = new IndicatorLight(hardwareMap,telemetry);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        Servo ejectServo = hardwareMap.get(Servo.class, "ejector");
        double lowerPos = 0;
        double ejectPos = 0.606;
        boolean isLowered = true;

        GamepadEvents controller1 = new GamepadEvents(gamepad1);

        waitForStart();

        while(opModeIsActive()){

            intake.setPower(controller1.left_stick_y);

            if (isLowered){
                ejectServo.setPosition(lowerPos);
            }
            else{
                ejectServo.setPosition(ejectPos);
            }

            if (controller1.a.onPress()){
                isLowered = !isLowered;
            }

            if(gamepad1.dpad_up){
                if (isLowered){
                    lowerPos -= 0.001;
                }else{
                    ejectPos -= 0.001;
                }
            }
            if(gamepad1.dpad_down){
                if (isLowered){
                    lowerPos += 0.001;
                }else{
                    ejectPos += 0.001;
                }
            }



            telemetry.addLine(colorSensor.toString());
            telemetry.addData("Servo Pos: ", ejectServo.getPosition());
            telemetry.addData("Servo isLowered: ",isLowered);
            telemetry.addData("Motor Power: ", intake.getPower());
            controller1.update();
            telemetry.update();
        }
    }
}
