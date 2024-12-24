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


        waitForStart();


    }
}
