package org.firstinspires.ftc.teamcode.utils.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {

    public enum Color{
        Yellow,
        Blue,
        Red,
        None
    }

    DigitalChannel pin0, pin1;


    public ColorSensor(HardwareMap hw){
        this(hw, "color0", "color1");
    }

    public ColorSensor(HardwareMap hw, String pin0Name, String pin1Name){
        this.pin0 = hw.digitalChannel.get(pin0Name);
        this.pin1 = hw.digitalChannel.get(pin1Name);
    }


    /**
     * Returns a the current color detected by the sensor
     * @return Returns a Color of Yellow, Blue, Red, or None.
     */
    public Color getColor(){
        boolean col0 = this.pin0.getState();
        boolean col1 = this.pin1.getState();

        if (col0 && col1){
            return Color.Yellow;
        }
        else if(col0){
            return Color.Blue;

        } else if (col1) {
            return Color.Red;
        }
        return Color.None;
    }

    /**
     * Takes multiple color samples and returns the value with the highest result count
     * @param repetitions Number of data samples requested
     * @return Final color determination
     */
    public Color getColorRepeat(double repetitions){
        int[] dataSamples = new int[Color.values().length];
        int maxIdx = dataSamples.length - 1;
        for(int i=0; i<repetitions; i++){
            int colorIdx = getColor().ordinal();
            dataSamples[colorIdx]++;
            if (dataSamples[maxIdx] < dataSamples[colorIdx]){
                maxIdx = colorIdx;
            }
        }
        return Color.values()[maxIdx];
    }

    public String toString(){
        return String.format("Pin 0: %b\n"+
                "Pin 1: %b\n" +
                "Color: %s\n",
                pin0.getState(),
                pin1.getState(),
                getColor());
    }
}
