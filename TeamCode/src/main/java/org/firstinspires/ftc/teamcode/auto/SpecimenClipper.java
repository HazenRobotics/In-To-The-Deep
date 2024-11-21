package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Clipper")
public class SpecimenClipper extends LinearOpMode {



    private Pose2d startPosition = new Pose2d(8.5,-61, Math.toRadians(-90));


    private Pose2d scoreSpecimen = new Pose2d(0, -25, Math.toRadians(-90));

    private Pose2d prepareToScore = calculateOffset(-90,-10, scoreSpecimen);

    private double ySpikeLineUp = -11;
    private double ySpikeHumanPlayerPush = -54;

    private double xSpikeLeft = 44;
    private double xSpikeMiddle = 53;
    private double xSpikeRight = 62;


    private double humanPlayerDelay = 1.5;
    private double closeClawDelay = 0.5;
    private double armRetractTime = 1;
    private double intakeMaxSpeed = 15;
    public ArrayList<String> telemetryLines = new ArrayList<String>();
    public ElapsedTime timer;

    private double armExtentionLength = 15;
    private Pose2d spikeRightActual = new Pose2d(41, -22, Math.toRadians(160));
    private Pose2d spikeRightLineUp = calculateOffset(20, armExtentionLength, spikeRightActual);
    //calculateOffset(160,-2,new Pose2d(-24, -35,Math.toRadians(160)));
    private Pose2d spikeRightIntake = calculateOffset(10, armExtentionLength-10,spikeRightActual);

    //Yellow Spike Middle
    private Pose2d spikeMiddleActual = new Pose2d(50, -22, Math.toRadians(0));
    private Pose2d spikeMiddleLineUp = calculateOffset(0,armExtentionLength, spikeMiddleActual);
    private Pose2d spikeMiddleIntake = calculateOffset(0,armExtentionLength-10, spikeMiddleActual);

    @Override
    public void runOpMode() throws InterruptedException {
        //Init phase
        FourEyesRobot robot = new FourEyesRobot(hardwareMap);
        MecanumDrive roadRunner = new MecanumDrive(hardwareMap, startPosition);
        timer = new ElapsedTime();
        waitForStart();
        //Start Phase
        timer.reset();
        robot.initializePowerStates();
        Actions.runBlocking(new ParallelAction(
                robot.autoPID(),
                new SequentialAction(
                        roadRunner.actionBuilder(startPosition)
                                //Prepare to score Preload
                                .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                                .strafeTo(prepareToScore.position)
                                .stopAndAdd(robot.waitForLiftArmPID(2))

                                //Score preload
                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Score Preload")))
                                .strafeTo(scoreSpecimen.position,new TranslationalVelConstraint(20))
                                .stopAndAdd(new InstantAction(robot::openClaw))
                                .strafeToLinearHeading(calculateOffset(-90,-20,scoreSpecimen).position, Math.toRadians(20))
//                                .lineToY(scoreSpecimen.position.y - 5)
                                .stopAndAdd(new InstantAction(robot::lowerClimb))
//


                                //Intake Right Sample
                                .strafeToLinearHeading(spikeRightLineUp.position, spikeRightLineUp.heading)
                                .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                                .stopAndAdd(new InstantAction(robot::toggleIntake))
                                .waitSeconds(armRetractTime)
                                .strafeToLinearHeading(spikeRightIntake.position,spikeRightIntake.heading,
                                        new TranslationalVelConstraint(intakeMaxSpeed))
                                //Deposit Sample to
                                .strafeToLinearHeading(new Vector2d(40,-48),Math.toRadians(-45))
                                .stopAndAdd(new InstantAction(robot::intakeBackward))
                                .stopAndAdd(new InstantAction(robot::toggleIntake))

                                //Intake Left Sample
                                .strafeToLinearHeading(spikeMiddleLineUp.position, spikeMiddleLineUp.heading)
                                .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                                .stopAndAdd(new InstantAction(robot::toggleIntake))
                                .waitSeconds(armRetractTime)
                                .strafeToLinearHeading(spikeMiddleIntake.position,spikeMiddleIntake.heading,
                                        new TranslationalVelConstraint(intakeMaxSpeed))
                                //Deposit Sample to
                                .strafeToLinearHeading(new Vector2d(40,-48),Math.toRadians(-45))
                                .stopAndAdd(new InstantAction(robot::intakeBackward))
                                .stopAndAdd(new InstantAction(robot::toggleIntake))


//                                .strafeToLinearHeading()
//                                //Drive to pushing position
//                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Pushing Left Sample")))
//                                .strafeTo(new Vector2d(36,-30))
//                                .strafeTo(new Vector2d(36,ySpikeLineUp))
//                                .strafeTo(new Vector2d(xSpikeLeft,ySpikeLineUp))
//                                .strafeTo(new Vector2d(xSpikeLeft,ySpikeHumanPlayerPush))
//
//                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Pushing Right Sample")))
//                                .strafeTo(new Vector2d(xSpikeLeft,ySpikeLineUp))
//                                .strafeTo(new Vector2d(xSpikeMiddle,ySpikeLineUp))
//                                .strafeTo(new Vector2d(xSpikeMiddle,ySpikeHumanPlayerPush))
//
//                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Intake First Specimen")))
//                                .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
//                                .strafeTo(new Vector2d(40,-49))
//                                .stopAndAdd(new InstantAction(robot::openClaw))
//                                .waitSeconds(humanPlayerDelay)
//                                .strafeTo(new Vector2d(40,-52.5),new TranslationalVelConstraint(20))
//                                .stopAndAdd(new InstantAction(robot::closeClaw))
//                                .waitSeconds(closeClawDelay)
//                                .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
//
//                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Score First Specimen")))
//                                .strafeTo(calculateOffset(180,3,prepareToScore).position)
//                                .stopAndAdd(robot.waitForLiftArmPID(2))
//                                .strafeTo(calculateOffset(180,3,scoreSpecimen).position,new TranslationalVelConstraint(20))
//                                .stopAndAdd(new InstantAction(robot::openClaw))
//                                .strafeTo(prepareToScore.position)
//                                .stopAndAdd(new InstantAction(robot::lowerClimb))

//
//
//                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Intake Second Specimen")))
//                                .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
//                                .strafeTo(new Vector2d(40,-49))
//                                .stopAndAdd(new InstantAction(robot::openClaw))
//                                .waitSeconds(humanPlayerDelay)
//                                .strafeTo(new Vector2d(40,-52.5),new TranslationalVelConstraint(20))
//                                .stopAndAdd(new InstantAction(robot::closeClaw))
//                                .waitSeconds(closeClawDelay)
//                                .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
//
//                                .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Score Second Specimen")))
//                                .strafeTo(calculateOffset(180,-3,prepareToScore).position)
//                                .stopAndAdd(robot.waitForLiftArmPID(2))
//                                .strafeTo(calculateOffset(180,-3,scoreSpecimen).position,new TranslationalVelConstraint(20))
//                                .stopAndAdd(new InstantAction(robot::openClaw))
//                                .strafeTo(prepareToScore.position)
////                                .strafeTo(new Vector2d(xSpikeMiddle,ySpikeLineUp))
////                                .strafeTo(new Vector2d(xSpikeRight,ySpikeLineUp))
////                                .strafeTo(new Vector2d(xSpikeRight,ySpikeHumanPlayerPush))
//
                                .build()

                )
        ));

    }
    public void addTelemetryMessage(String message){
        telemetryLines.add(message + " " + timer.time(TimeUnit.SECONDS) + " Elapsed");
        for (String info: telemetryLines) {
            telemetry.addLine(info);
        }
        telemetry.update();
    }

    public static Vector2d calculateOffset(double angleDegrees, double distance){
        return new Vector2d(-distance * Math.cos(Math.toRadians(angleDegrees)), -distance * Math.sin(Math.toRadians(angleDegrees)));
    }

    public static Pose2d calculateOffset(double angleDegrees, double distance, Pose2d target){
        Vector2d pos = target.position;
        return new Pose2d(pos.x - distance * Math.cos(Math.toRadians(angleDegrees)),pos.y - distance * Math.sin(Math.toRadians(angleDegrees)),Math.toRadians(angleDegrees));
    }
}
