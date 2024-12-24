package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;
import android.os.Build;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class IndicatorLight {
    private static final Map<Double, int[]> colorScale = new HashMap<>();
    static {
        colorScale.put(0.0, new int[]{0, 0, 0});        // Black
        colorScale.put(0.277, new int[]{255, 0, 0});    // Red
        colorScale.put(0.333, new int[]{255, 128, 0});  // Orange
        colorScale.put(0.388, new int[]{255, 255, 0});  // Yellow
        colorScale.put(0.444, new int[]{128, 255, 0});  // Sage
        colorScale.put(0.500, new int[]{0, 255, 0});    // Green
        colorScale.put(0.555, new int[]{0, 255, 128});  // Azure
        colorScale.put(0.611, new int[]{0, 255, 255});  // Blue
        colorScale.put(0.666, new int[]{128, 0, 255});  // Indigo
        colorScale.put(0.722, new int[]{255, 0, 255});  // Violet
        colorScale.put(1.0, new int[]{255, 255, 255});  // White
    }

    public static final double RED_WEIGHT = 0.277;
    public static final double GREEN_WEIGHT = 0.5;
    public static final double BLUE_WEIGHT = 0.611;
    public static final double YELLOW_WEIGHT = 0.388;
    Servo servo;
    Telemetry telemetry;

    public IndicatorLight(HardwareMap hw) {
        this(hw, "light");
    }
    public IndicatorLight(HardwareMap hw, String name) {
        servo = hw.get(Servo.class,name);
    }
    public IndicatorLight(HardwareMap hw, Telemetry t) {
        this(hw, "light");
        telemetry=t;
    }
    public double setColor(double color) {
        servo.setPosition(color);
        return color;
    }
    public double setColor(int[] rgb) {
        return setColor(rgbToFTC(rgb));


    }
    public static double rgbToFTC(int[] rgb) {
        // Normalize input RGB to [0, 1]
        double r = rgb[0] / 255.0;
        double g = rgb[1] / 255.0;
        double b = rgb[2] / 255.0;
        double minDistance = Double.MAX_VALUE;
        double closestFTC = 0.0;
        // Iterate over the predefined color scale
        for (Map.Entry<Double, int[]> entry : colorScale.entrySet()) {
            double ftc = entry.getKey();
            int[] referenceRGB = entry.getValue();
            // Normalize reference RGB to [0, 1]
            double rRef = referenceRGB[0] / 255.0;
            double gRef = referenceRGB[1] / 255.0;
            double bRef = referenceRGB[2] / 255.0;
            // Compute Euclidean distance
            double distance = Math.sqrt(Math.pow(r - rRef, 2) + Math.pow(g - gRef, 2) + Math.pow(b - bRef, 2));
            // Update closest FTC if this color is nearer
            if (distance < minDistance) {
                minDistance = distance;
                closestFTC = ftc;
            }
        }
        return closestFTC;
    }
    public double getColor() {
        return servo.getPosition();
    }

}
