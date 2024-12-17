package org.firstinspires.ftc.teamcode.auto.versionpepto;

import android.util.Size;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drivetrains.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.processors.SampleProcessor2;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Auto Tester")
public class AutoTesting extends LinearOpMode {

    private Pose2d startPosition = new Pose2d(-12,-64,Math.toRadians(90));



    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive roadRunner = new MecanumDrive(hardwareMap,startPosition);

        FourEyesRobot robot = new FourEyesRobot(hardwareMap);

        SampleProcessor2 visionProc = new SampleProcessor2(telemetry);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get( WebcamName.class, "Webcam 1"))
                .addProcessor(visionProc)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                robot.autoPID(),
                roadRunner.actionBuilder(startPosition)
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                        .build()
        ));

        //
//        Actions.runBlocking(new ParallelAction(
//                robot.autoPID(),
//                new SequentialAction(
//                roadRunner.actionBuilder(startPosition)
////                        .strafeTo( new Vector2d(-12,-40))
//                        .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
//                        .waitSeconds(5)
//                        .build()
//        )));
//        TrajectoryActionBuilder res = roadRunner.actionBuilder(startPosition)
//                .strafeTo( new Vector2d(-12,-40))
//
//                ;
//        res.endTrajectory();
    }
}
