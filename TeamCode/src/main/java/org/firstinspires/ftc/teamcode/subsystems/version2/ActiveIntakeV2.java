package org.firstinspires.ftc.teamcode.subsystems.version2;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.IndicatorLight;
import org.firstinspires.ftc.teamcode.utils.sensors.ColorSensor;

import java.util.HashMap;

//TODO: Conner Implements ActiveIntakeV2
public class ActiveIntakeV2 {


    DcMotorEx intake;
    Servo backDoor;
    boolean downDoor=true;

    ColorSensor colorSensor;
    IndicatorLight led;
    HardwareMap hardwareMap;

    public ActiveIntakeV2(HardwareMap hw) {
        hardwareMap=hw;
        intake = hw.get(DcMotorEx.class,"intake");
        backDoor = hw.get(Servo.class,"ejector");
        colorSensor = new ColorSensor(hardwareMap);
        led = new IndicatorLight(hardwareMap);

    }

    public void toggleBackDoor() {
//        downDoor=!downDoor;
        if(downDoor) {
//            backDoor.setPosition(0);
            bringUp();
        } else {
            dropDown();
//            backDoor.setPosition(0.606);
        }
    }
    public void dropDown() {
        backDoor.setPosition(0);
        downDoor=true;
    }
    public void bringUp() {
        backDoor.setPosition(0.489);
        downDoor=false;
    }
    public void setPower(double power) {
        intake.setPower(power);
        if(intake.getPower()!=0) {
            dropDown();
        }
    }

    public ColorSensor.Color checkIntakeState(){
        return colorSensor.getColor();
    }

    public void updateSensor(){
        ColorSensor.Color color = checkIntakeState();
        switch (color){
            case Yellow:
                led.setColor(IndicatorLight.YELLOW_WEIGHT);
                break;
            case Red:
                led.setColor(IndicatorLight.RED_WEIGHT + 0.01);
                break;
            case Blue:
                led.setColor(IndicatorLight.BLUE_WEIGHT);
                break;
            default:
                led.setColor(0);
                break;
        }
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Motor Power: %f\n" +
                "Ejector Up: %b\n" +
                "Color Detected: %s\n",
                intake.getPower(),
                downDoor,
                colorSensor.getColor());
    }
}

