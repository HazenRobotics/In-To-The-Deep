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

@Autonomous(name="SpecimenClipper2")
public class SpecimenClipper2 extends LinearOpMode {

    private Pose2d startPosition = new Pose2d(8.5,-59.5/*+0.25*/, Math.toRadians(-90));
    FourEyesRobot robot;
    MecanumDrive roadRunnerDrive;



    Pose2d barScoring = new Pose2d(2,-36,Math.toRadians(-90));
    Pose2d dropOff = new Pose2d(50, -40, Math.toRadians(-90));
    Pose2d innerSampleLineUp = new Pose2d(24, -32, Math.toRadians(23));
    Pose2d middleSampleLineUp = new Pose2d(36, -22, Math.toRadians(0));
    Pose2d outerSampleLineUp = new Pose2d(43, -22, Math.toRadians(0));
    Pose2d intakePosition = new Pose2d(49-9,-48,Math.toRadians(-90));
    ArrayList<String> telemetryLines;

    double clawOpenPrediction = 0.4;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new FourEyesRobot(hardwareMap);


        roadRunnerDrive = new MecanumDrive(hardwareMap, startPosition);

        telemetryLines = new ArrayList<>();
        timer = new ElapsedTime();

        waitForStart();

        timer.reset();
        robot.initializePowerStates();
        Actions.runBlocking(new ParallelAction(
                robot.autoPID(),
                new SequentialAction(
                   roadRunnerDrive.actionBuilder(startPosition)
                           .afterTime(29.75,new InstantAction(robot::lowerClimb))
                           .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                           .setReversed(true)
                           .strafeTo(barScoring.position.plus(new Vector2d(-2.5,0)))
                           .waitSeconds(0.05)
                           .lineToY(-28)
                           .stopAndAdd(new InstantAction(robot::openClaw))

                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Scoring Preload")))
                           .setReversed(false)

                           //Drive to inner Sample
                           .afterDisp(2,new InstantAction(robot::intakeSamplePos))
                           .afterDisp(3,new InstantAction(robot::toggleIntake))
                           .lineToY(-30)
                           .splineTo(innerSampleLineUp.position,innerSampleLineUp.heading.toDouble())
                           //Drive to inner Sample
                           //Drop off inner Sample to Human Player
                           //After traveling some path to pick up a sample, raise lift
                           .afterDisp(12,new InstantAction(robot::toggleIntake))
//                           .lineToX(37,
                           .splineTo(innerSampleLineUp.position.plus(new Vector2d(8,3.5)),Math.toRadians(15),
                                   (robotPose, _path, _disp) -> {
                                    if (robotPose.position.x.value() > 27){
                                        return 7;
                                    }else{
                                        return 40;
                                    }
                                   })
                           .splineTo(dropOff.position,dropOff.heading)
                           .stopAndAdd(new InstantAction(robot::intakeBackward))
                           .waitSeconds(0.1)
                           .stopAndAdd(new InstantAction(robot::deactivateIntake))
                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Depositing Sample 1")))
//                           .waitSeconds(0.3)

                           //Intake middle Sample
                           .setReversed(true)
                           .splineTo(middleSampleLineUp.position,middleSampleLineUp.heading.toDouble()-Math.toRadians(180))
                           .stopAndAdd(new InstantAction(robot::toggleIntake))
                           //Drop off middle Sample to Human Player
                           .setReversed(false)
                           .afterDisp(6,new InstantAction(robot::toggleIntake))//After traveling some path to pick up a sample
                           .lineToX(47,
                                   (robotPose, _path, _disp) -> {
                                       if (robotPose.position.x.value() > 37){
                                           return 10;
                                       }else{
                                           return 40;
                                       }
                                   })
                           .splineTo(dropOff.position.plus(new Vector2d(10,0)),dropOff.heading)
                           .stopAndAdd(new InstantAction(robot::intakeBackward))
                           .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
                           .stopAndAdd(new InstantAction(robot::openClaw))
                           .waitSeconds(0.1)
                           .stopAndAdd(new InstantAction(robot::deactivateIntake))
                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Depositing Sample 2")))


//                           //Intake Outer Sample
//                           .setReversed(true)
//                           .splineTo(outerSampleLineUp.position,outerSampleLineUp.heading.toDouble()-Math.toRadians(180))
//                           .stopAndAdd(new InstantAction(robot::toggleIntake))
//                           //Drop off Outer Sample to Human Player
//                           .setReversed(false)
//                           .lineToX(52,
//                                   new TranslationalVelConstraint(10))
//                           .afterDisp(5,new InstantAction(robot::toggleIntake))//After traveling some path to pick up a sample
//                           .splineTo(dropOff.position.plus(new Vector2d(10,0)),dropOff.heading,
//                                   new TranslationalVelConstraint(50))
//                           .stopAndAdd(new InstantAction(robot::intakeBackward))
//                           .waitSeconds(0.2)
//                           .stopAndAdd(new InstantAction(robot::deactivateIntake))
//                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Depositing Sample 3")))


//                           .splineToConstantHeading(intakePosition.position.plus(new Vector2d(0,-2)),Math.toRadians(-90))
//                           .waitSeconds(0.5)
                           .lineToY(intakePosition.position.y - 3,
                                   new TranslationalVelConstraint(15))
                           .stopAndAdd(robot::closeClaw)
                           .waitSeconds(0.05)
                           .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                           .setReversed(true)
                           .splineTo(barScoring.position.plus(new Vector2d(-2,8)),Math.toRadians(90))
//                           .stopAndAdd(robot.waitForLiftArmPID(1))
                           .afterTime(clawOpenPrediction,new InstantAction(robot::openWideClaw))
                           .lineToY(-22)
                           .stopAndAdd(new InstantAction(robot::openWideClaw))
//                           .waitSeconds(0.2)
                           .setReversed(false)
                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Scoring Specimen 1")))

                           .stopAndAdd(new InstantAction(robot::liftGoToZero))
                           .afterTime(0.2,new InstantAction(robot::intakeSpecimenPos))
                           .afterTime(0.4,new InstantAction(robot::openClaw))
                           .splineToConstantHeading(intakePosition.position.plus(new Vector2d(0,1)),Math.toRadians(-90))

//                           .waitSeconds(0.5)
                           .lineToY(intakePosition.position.y - 6,
                                   new TranslationalVelConstraint(10))
                           .stopAndAdd(robot::closeClaw)
                           .waitSeconds(0.05)
                           .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                           .setReversed(true)
                           .splineTo(barScoring.position.plus(new Vector2d(-1,2)),Math.toRadians(90))
//                           .stopAndAdd(robot.waitForLiftArmPID(1))
                           .afterTime(clawOpenPrediction,new InstantAction(robot::openWideClaw))
                           .lineToY(-22)
                           .stopAndAdd(new InstantAction(robot::liftGoToZero))

                           .setReversed(false)
                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Scoring Specimen 2")))


                           //Specimen 3
                           .afterTime(0.2,new InstantAction(robot::intakeSpecimenPos))
                           .afterTime(0.4,new InstantAction(robot::openClaw))

                           .splineToConstantHeading(intakePosition.position.plus(new Vector2d(0,3)),Math.toRadians(-90))

//                           .waitSeconds(0.5)
                           .lineToY(intakePosition.position.y - 6.5,
                                   new TranslationalVelConstraint(10))
                           .stopAndAdd(robot::closeClaw)
                           .waitSeconds(0.05)
                           .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                           .setReversed(true)
                           .splineTo(barScoring.position.plus(new Vector2d(0,7)),Math.toRadians(90))
//                           .stopAndAdd(robot.waitForLiftArmPID(1))
                           .afterTime(clawOpenPrediction,new InstantAction(robot::openWideClaw))
                           .lineToY(-22,new TranslationalVelConstraint(75))
                           .stopAndAdd(new InstantAction(robot::openWideClaw))
                           .setReversed(false)
                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Completed Scoring Specimen 3")))
                           .afterTime(0.2, new InstantAction(robot::lowerClimb))
                           .stopAndAdd(new InstantAction(robot::liftGoToZero))

//                           .afterDisp(2,new InstantAction(robot::lowerClimb))
//                           .strafeTo(intakePosition.position)
                           .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Complete!")))
                           .build()
                )));


    }
    public void addTelemetryMessage(String message){
        telemetryLines.add(message + " " + timer.time(TimeUnit.MILLISECONDS) + " Elapsed");
        for (String info: telemetryLines) {
            telemetry.addLine(info);
        }
        telemetry.update();
    }
}
