package org.firstinspires.ftc.teamcode.auto.versionpepto;

import android.util.Size;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drivetrains.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.versionpepto.Lift;
import org.firstinspires.ftc.teamcode.utils.sensors.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.utils.localization.LimeLightWrapper;
import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;
import org.firstinspires.ftc.teamcode.vision.processors.Sample;
import org.firstinspires.ftc.teamcode.vision.processors.SampleProcessor2;
import org.firstinspires.ftc.teamcode.vision.processors.Specimen;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="SpecimenClipper3")
public class SpecimenClipper3 extends LinearOpMode {

    boolean debugging = true;

    boolean enableCorrection = true;
    public boolean[] enabledSections = {
            true, //Preload

            true, //Inner Sample (1)

            true, //Middle Sample (2)

            true, //Intake Specimen 1

            true, //Deposit Specimen 1

            true, //Intake Specimen 2
            true, //Deposit Specimen 2

            true,//Intake Specimen 3
            true,//Deposit Specimen 3

            false, //Parking
    };

    public boolean missedSample = false;

    public Pose2d startPose = new Pose2d(9.5,-64.25,Math.toRadians(90));
    //        public Pose2d submersibleBar = new Pose2d(0,-31,Math.toRadians(-90));
//        public Pose2d specimenScorePose = new Pose2d(0,-24,Math.toRadians(-90));
        public Pose2d specimenScoreLineUp = new Pose2d(0,-36,Math.toRadians(90));
        public Pose2d specimenIntakeLineUp = new Pose2d(40, -45, Math.toRadians(-90));

    Pose2d dropOff = new Pose2d(50, -45, Math.toRadians(-90));
    //Locations relative to the Submersible
    //Sample Intake Positions
    public Pose2d innerSampleIntake, middleSampleIntake, outerSampleIntake;
    //Sample Line Up Positions
    public Pose2d innerSampleLineUp, middleSampleLineUp, outerSampleLineUp;

    public double armIntakeLength = 17;
    public void initSamplePositions(double y_sample_actual){
        //Sample Intake Positions
        innerSampleIntake = new Pose2d(49,y_sample_actual,Math.toRadians(20));
        middleSampleIntake = new Pose2d(59,y_sample_actual+ 0.25,Math.toRadians(0));
        outerSampleIntake = new Pose2d(69,y_sample_actual + 1 ,Math.toRadians(0));

        //Sample Line Up Positions
        innerSampleLineUp = new Pose2d(24,y_sample_actual - 9.5,Math.toRadians(20));
        middleSampleLineUp = new Pose2d(39,y_sample_actual + 0.25,Math.toRadians(0));
        outerSampleLineUp = new Pose2d(46,y_sample_actual + 1,Math.toRadians(0));
    }

    // default position, changes later o
    double submersibleBarPos = -22;
    double wallIntakePos = -55;
    double distToWall = 32;
    double llToWallX = dropOff.position.x, llToWallY = dropOff.position.y - 10;
    double specimenDepositGap = 2.5;

    double overclockVel = 70; //inches per second
    FourEyesRobot robot;
    MecanumDrive roadRunner;
    LimeLightWrapper limeLight;

    UltrasonicSensor distanceSensor;
    TelemetryRecorder telemetryRecorder;

    SampleProcessor2 visionProc;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new FourEyesRobot(hardwareMap);
        distanceSensor = new UltrasonicSensor(hardwareMap,"distanceSensor");
        distanceSensor.setDistanceOffset(UltrasonicSensor.distanceOffsets.BOT_POSITION_OFFSET);
        roadRunner = new MecanumDrive(hardwareMap,startPose);
        telemetryRecorder = new TelemetryRecorder(telemetry);
        limeLight = new LimeLightWrapper(hardwareMap.get(Limelight3A.class,"limelight"));

        visionProc = new SampleProcessor2(telemetry);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get( WebcamName.class, "Webcam 1"))
                .addProcessor(visionProc)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();


        initSamplePositions(-26);

        waitForStart();
        telemetryRecorder.resetTimer();
        robot.initializePowerStates();
        
//--------------------------------------------------------------------------------------------------
//-----------------------------------Score Preload--------------------------------------------------
//--------------------------------------------------------------------------------------------------
        if (enabledSections[0] && opModeIsActive()){
            //Raises Lift to began the distance check
//            Actions.runBlocking(new ParallelAction(
//                    robot.autoPID(),
//                    new SequentialAction(
//                            new InstantAction(robot::depositSpecimenPosForward),
//                            new SleepAction(0.25),
//                            robot.endPID(),
//                            telemetryRecorder.addInstantMessage("Ended Initial Check")
//                    )
//            ));
//            //Check the distance, take 10 samples to filter out faulty data
//            //Default assumption for distance to bar is 42
//            double distanceCheck = distanceSensor.distanceCheck(42,35,45,100);
//            //Should be around -22 for BarPos, -30 if calculating the bot's position
            submersibleBarPos = startPose.position.y + 42;
//            telemetryRecorder.addMessage("Sub recorded position: " + submersibleBarPos);
//            telemetryRecorder.addMessage("Beginning Preload Scoring\nBar position: "+(submersibleBarPos-8));
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(startPose)
                            .stopAndAdd(new InstantAction(robot::depositSpecimenPosForward))
                            .stopAndAdd(robot.waitForLiftArmPID(5,500,900,5000,5000))
                            //Travel to submersible bar based off distance
                            .lineToY(submersibleBarPos - 8,
                                    new TranslationalVelConstraint(overclockVel))
                            .stopAndAdd(new InstantAction(robot::openClaw))
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            double distanceCheck = distanceSensor.distanceCheck(8,4,12,50);
            submersibleBarPos = roadRunner.pose.position.y + distanceCheck;
            //Samples are approximately 4 inches away from the submersible in the y-direction
            initSamplePositions(submersibleBarPos - 4.75); //+Towards Sub -Towards Wall
            telemetryRecorder.addMessage("Sub recorded position: " + submersibleBarPos);
            telemetryRecorder.addMessage("Preload Scoring Complete!");
        }
//--------------------------------------------------------------------------------------------------
//-----------------------------------Intake Sample 1------------------------------------------------
//---------------------------------------(Inner)----------------------------------------------------
        if (enabledSections[1] && opModeIsActive()){
            telemetryRecorder.addMessage("Beginning Inner Sample Intake");
            telemetryRecorder.addMessage("Calculated y_coordinate to be: " + innerSampleIntake.position.y);
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            .stopAndAdd(new InstantAction(() -> robot.lift.goToPosition(Lift.LiftStates.SAMPLE_INTAKE)))
                            .afterDisp(2,new InstantAction(robot::VerticalArm))
                            //Backing away from the submersible
                            .lineToY(submersibleBarPos - 16)
                            .strafeToLinearHeading(new Vector2d(roadRunner.pose.position.x,submersibleBarPos-20),innerSampleLineUp.heading,
                                    new AngularVelConstraint(Math.PI * 0.5))
                            .afterDisp(0.01,new InstantAction(robot::intakeSamplePos))
                            .afterTime(0.1, new InstantAction(robot::toggleIntake))
                            //Lining up/Preparing to intake Sample
                            .strafeToLinearHeading(innerSampleLineUp.position,innerSampleLineUp.heading)
//                            .stopAndAdd(robot.waitForLiftArmPID(2))
//                            .waitSeconds(0.2)

                            //Intake Inner Sample
                            .lineToY(0.5 + innerSampleIntake.position.y - (armIntakeLength * Math.sin(innerSampleIntake.heading.toDouble())),
                                    new TranslationalVelConstraint(15))

                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            //Check for missed sample here!
            missedSample = checkForSample();
        }
//--------------------------------------------------------------------------------------------------
//---------------------------Intake Sample 1 Correction---------------------------------------------
//--------------------------------(Inner --> Middle)------------------------------------------------
        //If missed inner sample, go for middle
        if (enabledSections[1] && opModeIsActive() && missedSample){
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            //Pick up middle sample instead
                            .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                            .afterTime(0.1, new InstantAction(robot::toggleIntake))
                            .strafeToLinearHeading(middleSampleLineUp.position,middleSampleLineUp.heading)
                            .stopAndAdd(new InstantAction(robot::activateIntake))
                            //Intake middle sample
                            .lineToX( middleSampleIntake.position.x - armIntakeLength,
                                    new TranslationalVelConstraint(25))

                            //Then travel to Human Player Zone
                            .afterDisp(0.1, new InstantAction(robot::toggleIntake))
                            .afterDisp(0.1,telemetryRecorder.addInstantMessage("Completed Middle Sample Intake"))
                            .splineTo(dropOff.position, dropOff.heading,
                                    new TranslationalVelConstraint(overclockVel))
                            .stopAndAdd(robot::intakeBackward)
//                            .waitSeconds(0.1)
//                            .stopAndAdd(robot::intakeStop)
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
        }
        //If picked up inner sample, go to human player zone
        else if (enabledSections[1] && opModeIsActive()){
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)

                            .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Completed Inner Sample Intake"))
                            .splineTo(dropOff.position, dropOff.heading,
                                    new TranslationalVelConstraint(overclockVel))
                            .stopAndAdd(robot::intakeBackward)
//                            .waitSeconds(0.1)
//                            .stopAndAdd(robot::intakeStop)

                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            telemetryRecorder.addMessage("Completed Inner Sample Deposit");
        }

//--------------------------------------------------------------------------------------------------
//-----------------------------------Intake Sample 2------------------------------------------------
//-------------------------------------(Middle)-----------------------------------------------------
        //Intake Middle Sample assuming Sample 1 did not miss
        if (enabledSections[2] && opModeIsActive() && !missedSample){
            telemetryRecorder.addMessage("Beginning Middle Sample Intake");
            telemetryRecorder.addMessage("Calculated y_coordinate to be: " + middleSampleIntake.position.y);
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)

                            //Line up with middle sample position
                            .setReversed(true)
                            .splineToLinearHeading(middleSampleLineUp, middleSampleLineUp.heading)
                            .setReversed(false)
                            .stopAndAdd(new InstantAction(robot::toggleIntake))
                            .stopAndAdd(new InstantAction(robot::activateIntake))
                            //Intake middle sample
                            .lineToX( middleSampleIntake.position.x - armIntakeLength + 2)
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            telemetryRecorder.addMessage("Completed Middle Sample Deposit");
            //Check for missed sample here!
            missedSample = checkForSample();
        }
//--------------------------------------------------------------------------------------------------
//-----------------------------Intake Sample 2 Correction-------------------------------------------
//----------------------------------(Middle --> Outer)----------------------------------------------
        //If sample 1 or sample 2 missed
        //Intentional "if-if" as the previous section is capable
        //of changing the variable [missedSample] and triggering this correction path
        //If sample 1 or sample 2 is missed, we are forced to go for the Outer Sample
        if(enabledSections[2] && opModeIsActive() && missedSample){
            //Go for outer sample
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            //Travel to Outer Sample to Prepare Intake
                            .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                            .afterTime(0.1, new InstantAction(robot::toggleIntake))
                            .splineToLinearHeading(outerSampleLineUp, outerSampleLineUp.heading)
                            .lineToX( outerSampleIntake.position.x - armIntakeLength - 1.5,
                                    new TranslationalVelConstraint(10))
                            
                            //Raise intake to prevent dragging
                            .afterDisp(0.1, new InstantAction(robot::intakeSpecimenPos))
//                            .afterDisp(0.1, () -> robot.lift.goToPosition(Lift.LiftStates.SPECIMEN_INTAKE))
//                            .afterDisp(0.1, () -> robot.arm.goToPosition(Arm.ArmState.SPECIMEN_INTAKE))
                            .afterDisp(0.1,telemetryRecorder.addInstantMessage("Completed Outer Sample Intake"))
                            
                            //Drive to Human Player Zone

                            .strafeToLinearHeading(dropOff.position.plus(new Vector2d(-3,0)),dropOff.heading,
                                    new TranslationalVelConstraint(overclockVel))
//                            .stopAndAdd(robot::intakeBackward)
//                            .waitSeconds(0.1)
//                            .stopAndAdd(robot::intakeStop)
//                            .afterDisp(0.1,new InstantAction(robot::intakeStop))
//                            .strafeTo(dropOff.position.plus(new Vector2d(-7,0)),
//                                    new TranslationalVelConstraint(overclockVel))

//                            .stopAndAdd(new InstantAction(robot::intakeBackward))
//                            .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
        }
        //If picked up middle sample, go to human player zone
        else if (enabledSections[2] && opModeIsActive()){
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
//                            .stopAndAdd(() -> robot.lift.goToPosition(Lift.LiftStates.SPECIMEN_INTAKE))
//                            .stopAndAdd(() -> robot.arm.goToPosition(Arm.ArmState.SPECIMEN_INTAKE))
                            .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
                            .stopAndAdd(telemetryRecorder.addInstantMessage("Completed Middle Sample Intake"))
                            .splineTo(dropOff.position.plus(new Vector2d(-3,0)), dropOff.heading)
//                            .stopAndAdd(robot::intakeBackward)
////                            .waitSeconds(0.1)
////                            .stopAndAdd(robot::intakeStop)
//                            .afterDisp(0.1,new InstantAction(robot::intakeStop))
//                            .strafeTo(dropOff.position.plus(new Vector2d(-7,0)),
//                                    new TranslationalVelConstraint(overclockVel))
//                            .stopAndAdd(new InstantAction(robot::intakeBackward))
//                            .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
            telemetryRecorder.addMessage("Completed Inner Sample Deposit");
        }

//--------------------------------------------------------------------------------------------------
//------------------------------------Intake Specimen 1---------------------------------------------
//--------------------------------------------------------------------------------------------------
        //Intake Specimen
        if (enabledSections[3] && opModeIsActive()){
            telemetryRecorder.addMessage("Beginning Intaking Specimen 1");
//            Actions.runBlocking(new ParallelAction(
//                    robot.autoPID(),
//                    new SequentialAction(
//                            new InstantAction(robot::intakeStop),
//                            new InstantAction(robot::intakeSpecimenPos),
//                            new SleepAction(0.25),
//                            robot.endPID(),
//                            telemetryRecorder.addInstantMessage("Ended Initial Check")
//                    )
//            ));
            measureHumanPlayerZoneDistance();
            double botDistIntakeY = /*((llToWallY + distToWall) / 2.0)*/llToWallY - armIntakeLength;
            telemetryRecorder.addMessage("Dist: " + llToWallY/* + " " +distToWall*/);
            telemetryRecorder.addMessage("Travel Distance: " + botDistIntakeY);
            wallIntakePos = roadRunner.pose.position.y - botDistIntakeY;
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            .stopAndAdd(robot::intakeSpecimenPos)
                            .stopAndAdd(robot::openClaw)
                            .afterDisp(0.3, new InstantAction(robot::intakeBackward))
                            .afterDisp(1,new InstantAction(robot::intakeStop))
                            .lineToY(wallIntakePos)
                            .stopAndAdd(robot::closeClaw)
                            .stopAndAdd(robot.endPID())
                            .build()));
            telemetryRecorder.addMessage("Completed Intaking Specimen 1");
        }
//--------------------------------------------------------------------------------------------------
//------------------------------------Deposit Specimen 1---------------------------------------------
//--------------------------------------------------------------------------------------------------
        if (enabledSections[4] && opModeIsActive()){
            telemetryRecorder.addMessage("Beginning Deposit Specimen 1");
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
//                            .waitSeconds(0.2)
                            .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
//                            .afterTime(0.1,new InstantAction(() ->robot.wrist.goToPosition(Wrist.WristStates.SPECIMEN_DEPOSIT)))
                            .afterTime(0.1,new InstantAction(() -> robot.lift.getPid().setTarget(2400)))

                            .setReversed(true)
                            .splineToConstantHeading(specimenScoreLineUp.position,specimenScoreLineUp.heading)
                            .lineToY(submersibleBarPos-4)
                            .stopAndAdd(new InstantAction(robot::openClaw))
                            .stopAndAdd(new InstantAction(() -> robot.lift.goToPosition(Lift.LiftStates.SPECIMEN_INTAKE)))
                            .stopAndAdd(robot.endPID())
                            .build()));
            telemetryRecorder.addMessage("Completed Depositing Specimen 1");
        }

//--------------------------------------------------------------------------------------------------
//------------------------------------Intake Specimen 2---------------------------------------------
//--------------------------------------------------------------------------------------------------
        if(enabledSections[5] && opModeIsActive()){
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)

                            //Drive to Specimen
                            .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
                            .setReversed(false)
                            .splineToConstantHeading(
                                    specimenIntakeLineUp.position,
                                    specimenIntakeLineUp.heading,
                                    new TranslationalVelConstraint(overclockVel))
                            .afterDisp(0,robot::openClaw)
                            .lineToY(wallIntakePos)

                            //Intake Specimen
                            .stopAndAdd(robot::closeClaw)
                            .stopAndAdd(robot.endPID())
                            .build()));
            telemetryRecorder.addMessage("Completed Intaking Specimen 2");
        }
//--------------------------------------------------------------------------------------------------
//------------------------------------Deposit Specimen 2---------------------------------------------
//--------------------------------------------------------------------------------------------------
        if (enabledSections[6] && opModeIsActive()){
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
                            .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                            .setReversed(true)
                            .splineToConstantHeading(specimenScoreLineUp.position.plus(
                                    new Vector2d(specimenDepositGap,0))
                                    ,specimenScoreLineUp.heading)
                            .lineToY(submersibleBarPos-4)

                            .stopAndAdd(new InstantAction(robot::openClaw))
                            .stopAndAdd(new InstantAction(() -> robot.lift.goToPosition(Lift.LiftStates.SPECIMEN_INTAKE)))
                            .stopAndAdd(robot.endPID())
                            .build()));
            telemetryRecorder.addMessage("Completed Scoring Specimen 2");
        }
//--------------------------------------------------------------------------------------------------
//------------------------------------Intake Specimen 3---------------------------------------------
//--------------------------------------------------------------------------------------------------
        if(enabledSections[7] && opModeIsActive()){
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)

                            //Drive to Specimen
                            .stopAndAdd(new InstantAction(robot::intakeSpecimenPos))
                            .setReversed(false)
                            .splineToConstantHeading(
                                    specimenIntakeLineUp.position,
                                    specimenIntakeLineUp.heading,
                                    new TranslationalVelConstraint(overclockVel))
                            .afterDisp(0,robot::openClaw)
                            .lineToY(wallIntakePos)

                            //Intake Specimen
                            .stopAndAdd(robot::closeClaw)
                            .stopAndAdd(robot.endPID())
                            .build()));
            telemetryRecorder.addMessage("Completed Intaking Specimen 3");
        }
//--------------------------------------------------------------------------------------------------
//------------------------------------Deposit Specimen 3---------------------------------------------
//--------------------------------------------------------------------------------------------------
        //Check time for this, often times, we will be running low on it.
        if (enabledSections[8] && opModeIsActive()){
            double timeLeft = 30 - telemetryRecorder.getTime();


            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
//                            .afterTime(timeLeft-0.25,robot::lowerClimb)
                            .stopAndAdd(new InstantAction(robot::depositSpecimenPos))
                            .setReversed(true)
                            .splineToConstantHeading(specimenScoreLineUp.position.plus(
                                    new Vector2d(specimenDepositGap * 2,0)),Math.toRadians(90))
                            .lineToY(submersibleBarPos-4)
                            .stopAndAdd(new InstantAction(robot::openClaw))

                            .stopAndAdd(new InstantAction(robot::liftGoToZero))
                            .afterTime(0.3, new InstantAction(robot::lowerClimb))
                            .afterTime(0.3,telemetryRecorder.addInstantMessage("Completed Auto!"))
                            .build()));
        }


        //Keeps the OpMode running to observe telemetry data.
        telemetryRecorder.addMessage("Completed Auto!");
        while (debugging && opModeIsActive()){}
    }

    public void measureHumanPlayerZoneDistance(){
//        distToWall = distanceSensor.distanceCheck(26,20,30,100);

        Pose2d ll_pose = limeLight.getPosition();
        llToWallX = 72 - Math.abs(roadRunner.pose.position.x);
        llToWallY = 72 - Math.abs(roadRunner.pose.position.y);
        if (ll_pose != null && Math.abs(ll_pose.position.y) > 40 && Math.abs(ll_pose.position.y) < 50) {
            llToWallX = 72 - Math.abs(ll_pose.position.x);
            llToWallY = 72 - Math.abs(ll_pose.position.y);

        }
    }

    /**
     *
     * @return Returns a boolean on whether a sample is NOT in the intake.
     */
    public boolean checkForSample(){
        //If axon encoder says captured
        //And if camera says captured
        //Then sample was NOT missed
        //Otherwise, if one check fails, sample missed is TRUE


        boolean axonMiss = !robot.activeIntake.getSampleCaptured();
        boolean visionMiss = visionProc.getIntakeStatus() == Sample.NOTHING;

        telemetryRecorder.addMessage("Bool states: " + axonMiss + " " + visionProc.getIntakeString());

        if (!enableCorrection){
            return false;
        }

        return axonMiss && visionMiss;
    }
    
    public Pose2d addPose(Pose2d pos1, Pose2d pos2){
        return new Pose2d(
                pos1.position.x + pos2.position.y,
                pos1.position.y + pos2.position.y,
                pos1.heading.toDouble() + pos2.heading.toDouble()
        );
    }

}
