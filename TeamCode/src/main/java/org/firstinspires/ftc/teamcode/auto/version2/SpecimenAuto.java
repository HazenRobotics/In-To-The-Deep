package org.firstinspires.ftc.teamcode.auto.version2;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrains.Version2;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;

@Autonomous(name="5 Specimen Auto")
public class SpecimenAuto extends LinearOpMode {

    Pose2d startPosition = new Pose2d(8,-65,Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive roadRunner = new MecanumDrive(hardwareMap,startPosition);
        Version2 robot = new Version2(hardwareMap);
        TelemetryRecorder print = new TelemetryRecorder(telemetry);


        waitForStart();
        robot.completeInit();
        Actions.runBlocking(new ParallelAction(
            robot.autoPID(),
                roadRunner.actionBuilder(startPosition)
//                        .stopAndAdd(robot::specimenDeposit)
                        .lineToY(-34)
                        .waitSeconds(2)
//                        .stopAndAdd(robot::openClaw)
                        .lineToY(-44)

                        .build()
        ));


        while (opModeIsActive()){}
    }
}
