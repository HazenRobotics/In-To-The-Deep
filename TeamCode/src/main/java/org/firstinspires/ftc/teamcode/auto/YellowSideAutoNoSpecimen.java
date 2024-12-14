package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;

@Autonomous(name="YellowSideAutoNoSpecimen")
public class YellowSideAutoNoSpecimen extends LinearOpMode {


    public boolean debugging = true;

    public boolean[] enabledSections = {
            true, //Preload
            true, //Inner Sample
            true, //Middle Sample
            true, //Outer Sample
            true, //Parking
    };

    //Key Points of Interest
//        public Pose2d startPose = new Pose2d(-7.5,-60,Math.toRadians(-90));
    public Pose2d startPose = new Pose2d(-12,-66,Math.toRadians(180));
    //        public Pose2d submersibleBar = new Pose2d(0,-31,Math.toRadians(-90));
//        public Pose2d specimenScorePose = new Pose2d(0,-24,Math.toRadians(-90));
//        public Pose2d specimenScoreLineUp = new Pose2d(0,-36,Math.toRadians(-90));
    public Pose2d sampleScorePoseForward = new Pose2d(-53,-55,Math.toRadians(225));

    //Locations relative to the Submersible

    //Sample Intake Positions
    public Pose2d innerSampleIntake, middleSampleIntake, outerSampleIntake;
    //Sample Line Up Positions
    public Pose2d innerSampleLineUp, middleSampleLineUp, outerSampleLineUp;

    public double armIntakeLength = 17;

    public void initSamplePositions(double y_sample_actual){
        //Sample Intake Positions
        innerSampleIntake = new Pose2d(-49,y_sample_actual,Math.toRadians(160));
        middleSampleIntake = new Pose2d(-58,y_sample_actual,Math.toRadians(180));
        outerSampleIntake = new Pose2d(-69,y_sample_actual ,Math.toRadians(180));

        //Sample Line Up Positions
        innerSampleLineUp = new Pose2d(-24,y_sample_actual-9.5,Math.toRadians(160));
        middleSampleLineUp = new Pose2d(-38,y_sample_actual,Math.toRadians(180));
        outerSampleLineUp = new Pose2d(-43,y_sample_actual,Math.toRadians(180));
    }

    double submersibleBarPos = -22; // default position, changes later on

    private MecanumDrive roadRunner;

    TelemetryRecorder telemetryRecorder;
    UltrasonicSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        FourEyesRobot robot = new FourEyesRobot(hardwareMap);
        distanceSensor = new UltrasonicSensor(hardwareMap,"distanceSensor");
        distanceSensor.setDistanceOffset(UltrasonicSensor.distanceOffsets.BOT_POSITION_OFFSET);
        roadRunner = new MecanumDrive(hardwareMap,startPose);
        telemetryRecorder = new TelemetryRecorder(telemetry);
        initSamplePositions(submersibleBarPos - 3.75);


        waitForStart();

        telemetryRecorder.resetTimer();
        robot.initializePowerStates();

        //Score Preload
        if (enabledSections[0] && !isStopRequested()){
            //Raises Lift to began the distance check
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(startPose)
                            //Initializes robot's servos specifically
                    .stopAndAdd(new InstantAction(robot::activateIntake))

                    //Score Sample Pre-Load
                    .stopAndAdd(telemetryRecorder.addInstantMessage("Driving to Bucket..."))
                    .stopAndAdd(new InstantAction(robot::depositSamplePosForward)) //Set new target position
                    .stopAndAdd(new InstantAction(robot::VerticalArm)) //Override Arm Position
//                        .stopAndAdd(strafeWithSubsystems(startPosition, bucketPosition))
//                        .strafeTo(startPositionFromWall.position)
                    .strafeToLinearHeading(sampleScorePoseForward.position.plus(new Vector2d(-0.5,-2)), sampleScorePoseForward.heading)
                    .stopAndAdd(robot.waitForLiftArmPID(3))

                    //Lower Arm

                    .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                    .stopAndAdd(robot.waitForLiftArmPID(2))
                    .waitSeconds(0.5)

                    //Deposit Via Claw
                    .stopAndAdd(telemetryRecorder.addInstantMessage("Deposit Preload..."))
                    .stopAndAdd(new InstantAction(robot::intakeBackward))
                    .waitSeconds(0.1)
                    .stopAndAdd(telemetryRecorder.addInstantMessage("Stow Arm..."))
                    .stopAndAdd(new InstantAction(robot::deactivateIntake))
                    //Move arm temporarily
                    .stopAndAdd(new InstantAction(robot::VerticalArm))
//                        .stopAndAdd(robot.waitForLiftArmPID(1))
                    .stopAndAdd(telemetryRecorder.addInstantMessage("Stow Lift..."))
                    .stopAndAdd(new InstantAction(robot::liftGoToZero))
                    .stopAndAdd(robot.waitForLiftArmPID(2))



                            .stopAndAdd(robot.endPID())
                            .build()
            ));
        }
        //Intake Sample
        if (enabledSections[1] && !isStopRequested()){
            telemetryRecorder.addMessage("Beginning Inner Sample Intake");
            telemetryRecorder.addMessage("Calculated y_coordinate to be: " + innerSampleIntake.position.y);
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)

                            //Lining up/Preparing to intake Sample
                            .strafeToLinearHeading(innerSampleLineUp.position,innerSampleLineUp.heading)
                            .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                            .afterTime(0.1,new InstantAction(robot::toggleIntake))
                            .stopAndAdd(robot.waitForLiftArmPID(2))
                            .waitSeconds(0.25)

                            //Intake Inner Sample
                            .lineToY(innerSampleIntake.position.y-(armIntakeLength*Math.sin(innerSampleIntake.heading.toDouble())),
                                    new TranslationalVelConstraint(7.5))
                            .afterDisp(0.1, new InstantAction(robot::VerticalArm))
                            .afterDisp(0.1, new InstantAction(() ->robot.lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT)))
                            .afterDisp(0.1,telemetryRecorder.addInstantMessage("Completed Inner Sample Intake"))

                            //Drive to basket and score sample
                            .splineToConstantHeading(sampleScorePoseForward.position,sampleScorePoseForward.heading.plus(Math.toRadians(-45)))
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Ready to deposit!"))
                            .turnTo(sampleScorePoseForward.heading)
//                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
                            .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                            .waitSeconds(0.5)
                            .stopAndAdd(robot::intakeBackward)
                            .waitSeconds(0.1)
                            .stopAndAdd(robot::intakeStop)
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            telemetryRecorder.addMessage("Completed Inner Sample Deposit");
        }

        //Intake Middle Sample
        if (enabledSections[2] && !isStopRequested()){
            telemetryRecorder.addMessage("Beginning Middle Sample Deposit");
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            //Back up from bucket and lower lift and arm
                            .stopAndAdd(robot::VerticalArm)
                            .afterDisp(1,robot::liftGoToZero)
                            .afterDisp(3,robot::intakeSamplePos)
                            .afterDisp(5,robot::toggleIntake)

                            //Line up with middle sample position
                            .setReversed(true)
                            .splineToLinearHeading(middleSampleLineUp, middleSampleLineUp.heading)
                            .setReversed(false)
                            .stopAndAdd(robot.waitForLiftArmPID(2))

                            //Intake middle sample
                            .lineToX( middleSampleIntake.position.x + armIntakeLength - 1,
                                    new TranslationalVelConstraint(5))
                            .afterDisp(0.1, new InstantAction(robot::VerticalArm))
                            .afterDisp(0.1, new InstantAction(() ->robot.lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT)))

                            //Drive to basket and score sample
                            .splineToConstantHeading(sampleScorePoseForward.position,sampleScorePoseForward.heading.plus(Math.toRadians(-45)))
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Ready to deposit!"))
                            .turnTo(sampleScorePoseForward.heading)
//                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
                            .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                            .waitSeconds(0.5)
                            .stopAndAdd(robot::intakeBackward)
                            .waitSeconds(0.1)
                            .stopAndAdd(robot::intakeStop)
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            telemetryRecorder.addMessage("Completed Middle Sample Deposit");
        }

        if(enabledSections[3] && !isStopRequested()){
            telemetryRecorder.addMessage("Beginning Outer Sample Deposit");
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            //Back up from bucket and lower lift and arm
                            .stopAndAdd(robot::VerticalArm)
                            .afterDisp(1,robot::liftGoToZero)
                            .afterDisp(3,robot::intakeSamplePos)
                            .afterDisp(5,robot::toggleIntake)

                            //Line up with middle sample position
                            .setReversed(true)
                            .splineToLinearHeading(outerSampleLineUp, outerSampleLineUp.heading)
                            .setReversed(false)
                            .stopAndAdd(robot.waitForLiftArmPID(2))

                            //Intake middle sample
                            .lineToX( outerSampleIntake.position.x + armIntakeLength - 1,
                                    new TranslationalVelConstraint(5))
                            .afterDisp(0.1, new InstantAction(robot::VerticalArm))
                            .afterDisp(0.1, new InstantAction(() ->robot.lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT)))

                            //Drive to basket and score sample
                            .splineToConstantHeading(sampleScorePoseForward.position,sampleScorePoseForward.heading.plus(Math.toRadians(-45)))
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Ready to deposit!"))
                            .turnTo(sampleScorePoseForward.heading)
//                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
                            .stopAndAdd(new InstantAction(robot::depositSamplePosForward))
                            .waitSeconds(0.5)
                            .stopAndAdd(robot::intakeBackward)
                            .waitSeconds(0.1)
                            .stopAndAdd(robot::intakeStop)
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            telemetryRecorder.addMessage("Completed Outer Sample Deposit");
        }
        if (enabledSections[4] && !isStopRequested()){
            telemetryRecorder.addMessage("Beginning Parking");
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            .stopAndAdd(robot::VerticalArm)
                            .afterDisp(1,robot::lowerClimb)

                            .afterDisp(2, robot::raiseFlag)
                            .setTangent(90)
                            .splineToSplineHeading(new Pose2d(-24,-12,Math.toRadians(-90)),Math.toRadians(0))

                            .build()
            ));

            telemetryRecorder.addMessage("Completed Parking");
        }


        //Keeps the OpMode running to observe telemetry data.
        telemetryRecorder.addMessage("Completed Auto!");
        while (debugging && opModeIsActive()){}
    }
}
