package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * A Helper Class containing potentially useful methods to be shared across the repo
 */
public class MiscMethods {

    /**
     * Ensures the value remains within the set [min, max] range.
     * @param val Variable Value
     * @param min Minimum Possible Value
     * @param max Maximum Possible Value
     * @return
     */
    public static double clamp(double val, double min, double max){
        //Prevents the user from accidentally flipping the min and max values.
        if(min > max){
            double temp = max;
            max = min;
            min = temp;
        }
        return Math.max( min, Math.min(val , max));
    }

    public static Pose2d lerp(Pose2d anchor, Pose2d target, double dist){
        double deltaX = anchor.position.x - target.position.x;
        double deltaY = anchor.position.y - target.position.y;
        double heading = Math.atan2(deltaY, deltaX);

        return new Pose2d(
                anchor.position.x + dist * Math.cos(heading),
                anchor.position.y + dist * Math.sin(heading),
                heading
        );
    }

    public static Pose2d inverseLerp(Pose2d anchor, double dist, double angleDegree){
        double rad = Math.toRadians(angleDegree);
        return new Pose2d(
                anchor.position.x + Math.cos(rad) * dist,
                anchor.position.y + Math.sin(rad) * dist,
                rad
        );
    }
}
