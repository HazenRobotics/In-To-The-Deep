package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
import org.firstinspires.ftc.teamcode.utils.sensors.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Sensor Test", group = "Subsystem Tests")
public class DistanceTouchSensorTest extends LinearOpMode {

    Mecanum robot;

    DigitalChannel poloDistanceSensor;
    UltrasonicSensor distanceSensor;


    RevTouchSensor touchSensor;
//
//    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mecanum(hardwareMap);
        GamepadEvents controller1 = new GamepadEvents(gamepad1);
//
//        poloDistanceSensor = hardwareMap.get(DigitalChannel.class, "distance");
//        poloDistanceSensor.setMode(DigitalChannel.Mode.INPUT);
//        touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");
//
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        distanceSensor = new UltrasonicSensor(hardwareMap,"distanceSensor",5,0);

        distanceSensor.setDistanceOffset(UltrasonicSensor.distanceOffsets.NO_OFFSET);
        int currentEnum = 0;
        waitForStart();

        while(!isStopRequested()){

            robot.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (controller1.a.onPress()){
                currentEnum = (currentEnum + 1) % UltrasonicSensor.distanceOffsets.values().length;
            }

//            telemetry.addData("Distance (Dist Sensor): ",poloDistanceSensor.getState());
//            telemetry.addData("Touched (Touch Sensor): ",touchSensor.getValue());

            telemetry.addData("Distance CM:", distanceSensor.getDistance());
            telemetry.addData("Distance Inches:", distanceSensor.getDistanceInches());
            telemetry.addData("Offset: ",distanceSensor.getDistanceOffset());
            telemetry.update();
            controller1.update();
        }
    }
}
