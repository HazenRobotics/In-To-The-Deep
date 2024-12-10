/*
 *
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

@Config
public class UltrasonicSensor {

    public static final double CM_TO_INCHES = 0.3937;

    private AnalogInput distanceSensor;

    public double maxVoltage = 5; //default is 5 volts

    public double distanceOffset;

    public enum distanceOffsets{
        NO_OFFSET,
        SPECIMEN_INTAKE_OFFSET,
        HOOK_OFFSET,
        FRONT_OFFSET,
        BOT_POSITION_OFFSET
    }

    public HashMap<distanceOffsets,Double> offsets;


    public UltrasonicSensor (HardwareMap hw, double distanceOffset){
        this(hw, "distanceSensor", 5, distanceOffset);
    }

    public UltrasonicSensor(HardwareMap hw){
        this(hw, "distanceSensor", 5, 0);
    }
    public UltrasonicSensor(HardwareMap hw, String name){
        this(hw, name, 5,0);
    }

    public UltrasonicSensor(HardwareMap hw, String name, double maxVoltage, double distanceOffset){
        distanceSensor = hw.analogInput.get(name);
        this.maxVoltage = maxVoltage;
        this.distanceOffset = distanceOffset;
        initializeOffsets();
    }

    public UltrasonicSensor(HardwareMap hw, String name, double maxVoltage){
        this(hw, name, maxVoltage, 0);
    }

    public void initializeOffsets(){
        offsets = new HashMap<distanceOffsets, Double>();
        offsets.put(distanceOffsets.NO_OFFSET, 0.0);
        offsets.put(distanceOffsets.SPECIMEN_INTAKE_OFFSET,45.0);
        offsets.put(distanceOffsets.HOOK_OFFSET,21.0);
        offsets.put(distanceOffsets.FRONT_OFFSET,18.0);
        offsets.put(distanceOffsets.BOT_POSITION_OFFSET,-3.0);
    }



    public void setDistanceOffset(distanceOffsets offset){
        this.distanceOffset = offsets.get(offset);
    }

    public void setDistanceOffset(double distanceOffset){
        this.distanceOffset = distanceOffset;
    }

    public double getDistanceOffset(){
        return distanceOffset;
    }

    public double getDistanceOffsetValues(distanceOffsets offset){
        return offsets.get(offset);
    }

    public double getMaxVoltage(){
        return maxVoltage;
    }

    public double getVoltage(){
        return distanceSensor.getVoltage();
    }

    public double getDistance(){
        return getVoltage() * 520 / getMaxVoltage() - distanceOffset;
    }

    public double getRawDistance(){
        return getVoltage() * 520 / getMaxVoltage();
    }

    public double getDistanceInches(){
        return getDistance() * CM_TO_INCHES;
    }
}
