package org.firstinspires.ftc.teamcode.utils.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GoBuildaPinpoint implements LocalizerInterface {

    double WEIGHT = 1;

    GoBildaPinpointDriver pinpoint;

    public GoBuildaPinpoint(HardwareMap hw){
        this(hw, "odo");
    }

    public GoBuildaPinpoint(HardwareMap hw, String name){
        pinpoint = hw.get(GoBildaPinpointDriver.class, name);
        init();
    }

    public void init(){
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        pinpoint.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        pinpoint.recalibrateIMU();
        //pinpoint.resetPosAndIMU();
    }


    public void update(){
        pinpoint.update();
    }


    @Override
    public double getWeight() {
        return WEIGHT;
    }

    @Override
    public Pose2d getPosition() {
        update();
        Pose2D pos = pinpoint.getPosition();
        return new Pose2d(
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.RADIANS)
        );
    }

    @Override
    public boolean isValid() {
        return true;
    }

    @Override
    public void setColor(LimeLightWrapper.Color c) {}

    @Override
    public void setInitialPosition(Pose2d pose) {
        Pose2D position = new Pose2D(DistanceUnit.INCH,
                pose.position.x,
                pose.position.y,
                AngleUnit.RADIANS,
                pose.heading.toDouble());
        pinpoint.setPosition(position);
    }
}
