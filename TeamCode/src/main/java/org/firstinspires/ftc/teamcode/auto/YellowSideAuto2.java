package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;

@Autonomous(name="YellowSideAuto2")
public class YellowSideAuto2 extends LinearOpMode {


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
        public Pose2d startPose = new Pose2d(-11.75,-64.25,Math.toRadians(90));
//        public Pose2d submersibleBar = new Pose2d(0,-31,Math.toRadians(-90));
//        public Pose2d specimenScorePose = new Pose2d(0,-24,Math.toRadians(-90));
//        public Pose2d specimenScoreLineUp = new Pose2d(0,-36,Math.toRadians(-90));
        public Pose2d sampleScorePoseForward = new Pose2d(-51,-55.5,Math.toRadians(225));

        //Locations relative to the Submersible

    //Sample Intake Positions
    public Pose2d innerSampleIntake, middleSampleIntake, outerSampleIntake;
    //Sample Line Up Positions
    public Pose2d innerSampleLineUp, middleSampleLineUp, outerSampleLineUp;

    public double armIntakeLength = 17;

    public void initSamplePositions(double y_sample_actual){
        //Sample Intake Positions
        innerSampleIntake = new Pose2d(-49,y_sample_actual,Math.toRadians(160));
        middleSampleIntake = new Pose2d(-59,y_sample_actual-0.25,Math.toRadians(180));
        outerSampleIntake = new Pose2d(-69,y_sample_actual - 0.6 ,Math.toRadians(180));

        //Sample Line Up Positions
        innerSampleLineUp = new Pose2d(-24,y_sample_actual-9.5,Math.toRadians(160));
        middleSampleLineUp = new Pose2d(-39,y_sample_actual -0.25,Math.toRadians(180));
        outerSampleLineUp = new Pose2d(-43,y_sample_actual - 0.6,Math.toRadians(180));
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
        initSamplePositions(-26);


        waitForStart();

        telemetryRecorder.resetTimer();
        robot.initializePowerStates();

        //Score Preload
        if (enabledSections[0] && !isStopRequested()){
            //Raises Lift to began the distance check
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    new SequentialAction(
                            new InstantAction(robot::depositSpecimenPosForward),
                            new SleepAction(0.5),
                            robot.endPID(),
                            telemetryRecorder.addInstantMessage("Ended Initial Check")
                    )
            ));
            //Check the distance, take 10 samples to filter out faulty data
            //Default assumption for distance to bar is 42
            double distanceCheck = distanceSensor.distanceCheck(42,35,45,100);
            //Should be around -22 for BarPos, -30 if calculating the bot's position
            submersibleBarPos = startPose.position.y + distanceCheck;
            telemetryRecorder.addMessage("Sub recorded position: " + submersibleBarPos);
            telemetryRecorder.addMessage("Beginning Preload Scoring\nBar position: "+(submersibleBarPos-8));
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(startPose)
                            .stopAndAdd(new InstantAction(robot::depositSpecimenPosForward))
                            .stopAndAdd(robot.waitForLiftArmPID(5))
                            //Travel to submersible bar based off distance
                            .lineToY(submersibleBarPos - 8)
                            .stopAndAdd(new InstantAction(robot::openClaw))
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            distanceCheck = distanceSensor.distanceCheck(8,4,12,100);
            submersibleBarPos = roadRunner.pose.position.y + distanceCheck;
            //Samples are approximately 4 inches away from the submersible in the y-direction
            initSamplePositions(submersibleBarPos - 4.5); //+Towards Sub -Towards Wall
            telemetryRecorder.addMessage("Sub recorded position: " + submersibleBarPos);
            telemetryRecorder.addMessage("Preload Scoring Complete!");
        }
        //Intake Sample
        if (enabledSections[1] && !isStopRequested()){
            telemetryRecorder.addMessage("Beginning Inner Sample Intake");
            telemetryRecorder.addMessage("Calculated y_coordinate to be: " + innerSampleIntake.position.y);
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            .afterDisp(2,new InstantAction(robot::VerticalArm))
                            //Backing away from the submersible
                            .lineToY(submersibleBarPos - 16)

                            //Lining up/Preparing to intake Sample
                            .strafeToLinearHeading(innerSampleLineUp.position,innerSampleLineUp.heading)
                            .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                            .afterTime(0.1,new InstantAction(robot::toggleIntake))
                            .stopAndAdd(robot.waitForLiftArmPID(2))
                            .waitSeconds(0.25)

                            //Intake Inner Sample
                            .lineToY(0.5 + innerSampleIntake.position.y-(armIntakeLength*Math.sin(innerSampleIntake.heading.toDouble())),
                                    new TranslationalVelConstraint(7.5))
                            .afterDisp(0.1, new InstantAction(robot::VerticalArm))
                            .afterDisp(0.1, new InstantAction(() ->robot.lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT)))
                            .afterDisp(0.1,telemetryRecorder.addInstantMessage("Completed Inner Sample Intake"))

                            //Drive to basket and score sample
                            .splineTo(sampleScorePoseForward.position,sampleScorePoseForward.heading)
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Ready to deposit!"))
                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
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
                            .splineTo(sampleScorePoseForward.position,sampleScorePoseForward.heading)
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Ready to deposit!"))
                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
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
                            .splineTo(sampleScorePoseForward.position,sampleScorePoseForward.heading)
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Ready to deposit!"))
                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
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
