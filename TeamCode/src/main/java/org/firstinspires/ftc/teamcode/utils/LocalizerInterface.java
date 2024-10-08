package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * An Interface to easily cycle through localizer data.
 */
public interface LocalizerInterface {

    // This is a value to determine how accurate this localizer's data is
    public double getWeight();

    /* For our purposes, we only need 3 data points:
        - X Position (Inches)
        - Y Position (Inches)
        - Rotation (Radians)
    */
    public Pose2d getPosition();


}
