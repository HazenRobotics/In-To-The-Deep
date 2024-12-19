package org.firstinspires.ftc.teamcode.auto.versionpepto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrains.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

//@Autonomous(name = "YellowSideAutoPreloadClipper")
public class YellowSideAutoPreloadClipper extends LinearOpMode {

    private FourEyesRobot robot;
    private MecanumDrive roadRunnerDrive;

    //Subsystem info
    double armExtentionLength = 15;

    //Locations
    private Pose2d startPosition = new Pose2d(-8.5,-61, Math.toRadians(-90));

    //Sample Offsets
    private double sampleOffset = 4;
    private Pose2d bucketPosition = new Pose2d(-52.5, -57.5 + sampleOffset, Math.toRadians(225));


    double spikeYOffset = 0;
    double spikeXOffset = 3;
    //Yellow Spike Right
    private Pose2d yellowSpikeRightActual = new Pose2d(-41 + spikeXOffset, -28.5 + spikeYOffset+ sampleOffset, Math.toRadians(160));
    private Pose2d yellowSpikeRightLineUp = calculateOffset(160, armExtentionLength, yellowSpikeRightActual);
    //calculateOffset(160,-2,new Pose2d(-24, -35,Math.toRadians(160)));
    private Pose2d yellowSpikeRightIntake = calculateOffset(170, armExtentionLength-10,yellowSpikeRightActual);

    //Yellow Spike Middle
    private Pose2d yellowSpikeMiddleActual = new Pose2d(-50+ spikeXOffset, -27 + spikeYOffset+ sampleOffset, Math.toRadians(180));
    private Pose2d yellowSpikeMiddleLineUp = calculateOffset(180,armExtentionLength, yellowSpikeMiddleActual);

    private Pose2d yellowSpikeMiddleIntake = calculateOffset(180,armExtentionLength-10, yellowSpikeMiddleActual);


    //Yellow Spike Left
    private Pose2d yellowSpikeLeftActual = new Pose2d(-59+ spikeXOffset, -27.5 + spikeYOffset+ sampleOffset, Math.toRadians(180));

    private Pose2d yellowSpikeLeftLineUp = calculateOffset(180, armExtentionLength, yellowSpikeLeftActual);
    private Pose2d yellowSpikeLeftIntake = calculateOffset(180,armExtentionLength-10, yellowSpikeLeftActual);

    private double intakeMaxSpeed = 30;
    private ArrayList<String> telemetryLines = new ArrayList<String>();

    private double depositTime = 0.25;
    private double armScoreTime = 1;

    private double armRetractTime = 1;

    private Pose2d scoreSpecimen = new Pose2d(0, -25, Math.toRadians(-90));
    private Pose2d prepareToScore = calculateOffset(-90,-10, scoreSpecimen);


    private ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        //Init robot
//        Lift lift = new Lift(hardwareMap);
//        Arm arm = new Arm(hardwareMap);



        robot = new FourEyesRobot(hardwareMap);


        roadRunnerDrive = new MecanumDrive(hardwareMap, startPosition);

        waitForStart();
        //Auto Begins
        timer.reset();


        Actions.runBlocking(new ParallelAction(
                robot.autoPID(),
                roadRunnerDrive.actionBuilder(startPosition)
                        //Initializes robot's servos specifically
                        .stopAndAdd(new InstantAction(robot::initializePowerStates))
                        .stopAndAdd(new InstantAction(robot::activateIntake))

                        //Prepare to score Preload
                        .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                        .strafeTo(prepareToScore.position)
                        .stopAndAdd(robot.waitForLiftArmPID(2))

                        //Score preload
                        .stopAndAdd(new InstantAction(() -> addTelemetryMessage("Score Preload")))
                        .strafeTo(scoreSpecimen.position,new TranslationalVelConstraint(20))
                        .stopAndAdd(new InstantAction(robot::openClaw))
                        .strafeToLinearHeading(calculateOffset(-90,-20,scoreSpecimen).position, Math.toRadians(-90))
//                                .lineToY(scoreSpecimen.position.y - 5)
                        .stopAndAdd(new InstantAction(robot::VerticalArm))
                        .stopAndAdd(new InstantAction(robot::closeClaw))
                        .stopAndAdd(new InstantAction(robot::lowerClimb))

                        //Intake Right Sample
                        .strafeToLinearHeading(yellowSpikeRightLineUp.position, yellowSpikeRightLineUp.heading)
                        .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                        .stopAndAdd(new InstantAction(robot::toggleIntake))
                        .waitSeconds(armRetractTime)
                        .strafeToLinearHeading(yellowSpikeRightIntake.position,yellowSpikeRightIntake.heading,
                                new TranslationalVelConstraint(intakeMaxSpeed/2))


                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Driving to Bucket...")))
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward)) //Set new target position
                        .stopAndAdd(new InstantAction(robot::VerticalArm)) //Override Arm Position
//                        .stopAndAdd(strafeWithSubsystems(startPosition, bucketPosition))
                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(0.5,-0.6)), bucketPosition.heading)
                        .stopAndAdd(robot.waitForLiftArmPID(5))
                        //Lower Arm
                        .waitSeconds(armScoreTime)
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                        .stopAndAdd(robot.waitForLiftArmPID(2))
                        .waitSeconds(0.1)

                        //Deposit Via Claw
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Deposit Preload...")))
                        .stopAndAdd(new InstantAction(robot::intakeBackward))
                        .waitSeconds(depositTime)
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Stow Arm...")))
                        .stopAndAdd(new InstantAction(robot::deactivateIntake))

                        //Move arm temporarily
                        .stopAndAdd(new InstantAction(robot::VerticalArm))
                        .stopAndAdd(robot.waitForLiftArmPID(1))
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Stow Lift...")))
                        .stopAndAdd(new InstantAction(robot::liftGoToZero))
                        .stopAndAdd(robot.waitForLiftArmPID(3))


                        //Intake Middle Spike Mark
                        .strafeToLinearHeading(yellowSpikeMiddleLineUp.position, yellowSpikeMiddleLineUp.heading)
                        .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                        .stopAndAdd(new InstantAction(robot::toggleIntake))
                        .waitSeconds(armRetractTime)
                        .lineToX(yellowSpikeMiddleIntake.position.x + 2,new TranslationalVelConstraint(intakeMaxSpeed/2))
//                        .strafeToLinearHeading(yellowSpikeMiddleIntake.position,yellowSpikeMiddleIntake.heading,
//                                new TranslationalVelConstraint(intakeMaxSpeed))


                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Driving to Bucket...")))
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward)) //Set new target position
                        .stopAndAdd(new InstantAction(robot::VerticalArm)) //Override Arm Position
                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(0.5,-0.6)), bucketPosition.heading)
                        .stopAndAdd(robot.waitForLiftArmPID(3))
                        //Lower Arm
                        .waitSeconds(armScoreTime)
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                        .stopAndAdd(robot.waitForLiftArmPID(2))
                        .waitSeconds(0.1)

                        //Deposit Via Claw
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Deposit Preload...")))
                        .stopAndAdd(new InstantAction(robot::intakeBackward))
                        .waitSeconds(depositTime)
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Stow Arm...")))
                        .stopAndAdd(new InstantAction(robot::deactivateIntake))

                        //Move arm temporarily
                        .stopAndAdd(new InstantAction(robot::VerticalArm))
                        .stopAndAdd(robot.waitForLiftArmPID(1))
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Stow Lift...")))
                        .stopAndAdd(new InstantAction(robot::liftGoToZero))
                        .stopAndAdd(robot.waitForLiftArmPID(3))



                        .strafeToLinearHeading(yellowSpikeLeftLineUp.position, yellowSpikeLeftLineUp.heading)
                        .stopAndAdd(new InstantAction(robot::intakeSamplePos))
//                        .stopAndAdd(new InstantAction(robot::openClaw))
                        .stopAndAdd(new InstantAction(robot::toggleIntake))
                        .waitSeconds(armRetractTime)
                        .lineToX(yellowSpikeLeftIntake.position.x+4,new TranslationalVelConstraint(intakeMaxSpeed/2))
//                        .strafeToLinearHeading(yellowSpikeLeftIntake.position,yellowSpikeLeftIntake.heading,
//                                new TranslationalVelConstraint(intakeMaxSpeed))
                        .lineToX(yellowSpikeLeftIntake.position.x-4)
//                        .strafeTo(calculateOffset(180,armExtentionLength-5, yellowSpikeLeftActual).position)

                        //Deposit Left Spike Mark
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Driving to Bucket...")))
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward)) //Set new target position
                        .stopAndAdd(new InstantAction(robot::VerticalArm)) //Override Arm Position
                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(0,-2)), bucketPosition.heading)
                        .stopAndAdd(robot.waitForLiftArmPID(4))

                        //Lower Arm
                        .waitSeconds(armScoreTime)
                        .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                        .stopAndAdd(robot.waitForLiftArmPID(2))
                        .waitSeconds(0.1)

                        //Deposit Via Claw
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Deposit Preload...")))
                        .stopAndAdd(new InstantAction(robot::intakeBackward))
                        .waitSeconds(depositTime)
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Stow Arm...")))
                        .stopAndAdd(new InstantAction(robot::deactivateIntake))

                        //Move arm temporarily
                        .stopAndAdd(new InstantAction(robot::VerticalArm))
                        .stopAndAdd(robot.waitForLiftArmPID(1))
                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Stow Lift...")))

                        .stopAndAdd(new InstantAction(robot::lowerClimb))
                        .stopAndAdd(new InstantAction(robot::raiseFlag))
                        .setTangent(90)
                        .splineToSplineHeading(new Pose2d(-24,-12,Math.toRadians(-90)),Math.toRadians(0),
                                new TranslationalVelConstraint(80))

                        .stopAndAdd(new InstantAction ( () ->addTelemetryMessage("Complete! ")))
                        .build()

        ));
    }

    public ParallelAction strafeWithSubsystems(Pose2d startPosition, Vector2d endPosition){
        return strafeWithSubsystems(startPosition, new Pose2d(endPosition,startPosition.heading));
    }

    public ParallelAction strafeWithSubsystems(Pose2d startPosition, Pose2d endPosition){
        return new ParallelAction(
                //Travel to place to deposit
                roadRunnerDrive.actionBuilder(startPosition)
                        .strafeToLinearHeading(endPosition.position.plus(
                                calculateOffset(180, armExtentionLength)
                        ), endPosition.heading)
                        .build(),
                robot.waitForLiftArmPID(6) //Ensure that subsystems are in the right position
        );
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

    @SuppressLint("DefaultLocale")
    public String poseToString(Pose2d pos){
        return String.format("%f, %f, %f",
                pos.position.x,
                pos.position.y,
                pos.heading.toDouble());
    }

    public Pose2d duplicatePose(Pose2d pos){
        return new Pose2d(pos.position, pos.heading);
    }
}
