//package org.firstinspires.ftc.teamcode.auto.version2;
//
//import android.annotation.SuppressLint;
//
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.drivetrains.FourEyesRobot;
//import org.firstinspires.ftc.teamcode.drivetrains.Version2;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArm;
//import org.firstinspires.ftc.teamcode.subsystems.version2.DepositLift;
//import org.firstinspires.ftc.teamcode.subsystems.version2.ExtendoSlide;
//import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
//import org.firstinspires.ftc.teamcode.utils.MiscMethods;
//import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;
//
//import java.util.ArrayList;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "YellowSideAuto")
//public class YellowSideAutoV2 extends LinearOpMode {
//
//    private Version2 robot;
//    private MecanumDrive roadRunnerDrive;
//
//    //Subsystem info
//    double armExtentionLength = 15;
//
//    //Locations
//    public static final Pose2d startPosition = new Pose2d(-39,-65,Math.toRadians(90));
//    public static final Pose2d sample1postion = new Pose2d(-59, -58, Math.toRadians(90));
//    public static final Pose2d sample2postion = new Pose2d(-57.5, -48, Math.toRadians(75));
//    public static final Pose2d sample3postion = new Pose2d(-48, -48, Math.toRadians(45));
//    public static final Pose2d bucketPosition = new Pose2d(-57, -61, Math.toRadians(45));
//
//    Pose2d innerIntake = MiscMethods.lerp(bucketPosition,new Pose2d(-49,25,0),0.1);
//    Pose2d middleIntake = MiscMethods.lerp(bucketPosition,new Pose2d(-59,25,0),0.1);
//
//    Pose2d outerLineUp = new Pose2d(-54, -49,Math.toRadians(90));
//    Pose2d outerIntake = MiscMethods.lerp(outerLineUp, new Pose2d(-69, -24, 0), 0.1 );
//    Pose2d submerisbleIntake = new Pose2d(-24, -12, 0);
//
//    double spikeYOffset = 0;
//    double spikeXOffset = 3;
//    //Yellow Spike Right
//
//    private double intakeMaxSpeed = 30;
//    private ArrayList<String> telemetryLines = new ArrayList<String>();
//
//    private double depositTime = 0.25;
//    private double armScoreTime = 1;
//
//    private double armRetractTime = 1.5;
//
//    private ElapsedTime timer = new ElapsedTime();
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //Init robot
////        Lift lift = new Lift(hardwareMap);
////        Arm arm = new Arm(hardwareMap);
//
//
//
//        robot = new Version2(hardwareMap);
//
//
//        roadRunnerDrive = new MecanumDrive(hardwareMap, startPosition);
//        TelemetryRecorder print = new TelemetryRecorder(telemetry);
//
//
//
//
//        robot.specAutoInit();
//        robot.closeClaw();
//        waitForStart();
//        //Auto Begins
//        timer.reset();
//        print.resetTimer();
//
//
//        Actions.runBlocking(new ParallelAction(
//                robot.autoPID(),
//                roadRunnerDrive.actionBuilder(startPosition)
//                        //Score Preload
//                        .lineToY(bucketPosition.position.y)
//                        .stopAndAdd(robot::sampleDeposit)
//                        .waitSeconds(0.5)
//                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(-1,-1)), Math.toRadians(90))
//                        .stopAndAdd(robot.waitForLift(2))
//                        .turnTo(bucketPosition.heading)
//                        .stopAndAdd(new InstantAction(() ->robot.arm.goToPosition(IntakeArm.IntakeArmStates.INTAKE)))
//                        .stopAndAdd(robot::openClaw)
//
//
//                        //Score Sample 1
//                        .strafeToLinearHeading(new Vector2d(-47,-50),Math.toRadians(90))
//                        .stopAndAdd(new InstantAction(robot::sampleIntake))
//                        .waitSeconds(armRetractTime)
//                        .stopAndAdd(robot::transferPosition)
//                        .stopAndAdd(new InstantAction(()->{
//                            robot.deposit.openClaw();
//                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE);
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.TRASNFER);
//                        }))
//                        .afterTime(armRetractTime/2,robot::transferPosition)
//                        .waitSeconds(armRetractTime)
//                        .stopAndAdd(robot::closeClaw)
//                        .waitSeconds(0.1)
//                        .stopAndAdd(robot::sampleDeposit)
//                        .afterTime(0.5,new InstantAction(()-> {
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.INTAKE);
//                            robot.reverseIntake();
//                        }))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(1,-1)), bucketPosition.heading)
//                        .stopAndAdd(robot::openClaw)
//
//                        //Score Sample 2
//                        .strafeToLinearHeading(new Vector2d(-58,-50),Math.toRadians(90), new AngularVelConstraint(Math.PI / 8))
//                        .stopAndAdd(new InstantAction(robot::sampleIntake))
//                        .waitSeconds(armRetractTime)
//                        .stopAndAdd(new InstantAction(()->{
//                            robot.deposit.openClaw();
//                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE);
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.TRASNFER);
//                        }))
//                        .afterTime(armRetractTime/2,robot::transferPosition)
//                        .waitSeconds(armRetractTime)
//                        .stopAndAdd(robot::closeClaw)
//                        .waitSeconds(0.1)
//                        .stopAndAdd(robot::sampleDeposit)
//                        .afterTime(0.5,new InstantAction(()-> {
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.INTAKE);
//                        robot.reverseIntake();
//                        }))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(2,-2)), bucketPosition.heading)
//                        .stopAndAdd(robot::openClaw)
//
//                        //Score Sample 3
//                        .strafeToLinearHeading(outerLineUp.position,outerIntake.heading.plus(Math.toRadians(180)))
//                        .stopAndAdd(new InstantAction(robot::sampleIntake))
//                        .waitSeconds(armRetractTime)
//                        .stopAndAdd(new InstantAction(()->{
//                            robot.deposit.openClaw();
//                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE);
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.TRASNFER);
//                        }))
//                        .afterTime(armRetractTime/2,robot::transferPosition)
//                        .stopAndAdd(new InstantAction(() ->robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER)))
//                        .waitSeconds(armRetractTime)
//                        .stopAndAdd(robot::closeClaw)
//                        .waitSeconds(0.1)
//                        .stopAndAdd(robot::sampleDeposit)
////                        .afterTime(0.5,new InstantAction(()-> robot.arm.goToPosition(IntakeArm.IntakeArmStates.INTAKE)))
//                        .waitSeconds(1)
//                        .strafeToLinearHeading(bucketPosition.position.plus(new Vector2d(2,-2)), bucketPosition.heading)
//                        .stopAndAdd(robot::openClaw)
//
//                        .waitSeconds(0.1)
//
//                        .afterDisp(2, new InstantAction(() -> {
//                            robot.lift.goToPosition(DepositLift.LiftStates.TRANSFER);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.PARK_ARM);
//                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
//                        }))
//                        .splineToLinearHeading(new Pose2d(submerisbleIntake.position, Math.toRadians(180)), Math.toRadians(0))
//
//
//
//
////                        //Score Sample Pre-Load
////                        .stopAndAdd(new InstantAction(() -> this.addTelemetryMessage("Driving to Bucket...")))
////
////
////                        .stopAndAdd(new InstantAction ( () ->addTelemetryMessage("Complete! ")))
//                        .build()
//
//        ));
//
//
//
//
//
//
////        Actions.runBlocking(
////                roadRunnerDrive.actionBuilder(startPosition)
////                .strafeToLinearHeading(bucketPosition,Math.toRadians(180))
////                .build());
//        //This is the primary action loop.
//
////        Actions.runBlocking(new ParallelAction(
////                robot.autoPID(), //This PID loop will be constantly running in the background
////                new SequentialAction(
////                        //Initializes robot's servos specifically
////                        new InstantAction(robot::initializePowerStates),
////                        new InstantAction(robot::activateIntake),
////                        //Score Sample Pre-Load
////                        new InstantAction(() -> this.addTelemetryMessage("Driving to Bucket...")),
////                        new InstantAction(robot::depositSamplePosForward), //Set new target position
////                        new InstantAction(robot::VerticalArm), //Override Arm Position
////                        strafeWithSubsystems(startPosition, bucketPosition),
////                        //Lower Arm
////                        new InstantAction(robot::depositSamplePosForward),
////                        robot.waitForLiftArmPID(2),
////                        //Deposit Via Claw
////                        new InstantAction(() -> this.addTelemetryMessage("Deposit Preload...")),
////                        new InstantAction(robot::intakeBackward),
////                        new SleepAction(1), //Wait for claw to completely open and deposit
////                        new InstantAction(() -> this.addTelemetryMessage("Stow Arm...")),
////                        new InstantAction(robot::deactivateIntake),
////                        //Move arm temporarily
////                        new InstantAction(robot::VerticalArm),
////                        robot.waitForLiftArmPID(1),
////                        new InstantAction(() -> this.addTelemetryMessage("Stow Lift...")),
////                        new InstantAction(robot::resetLift),
////                        robot.waitForLiftArmPID(3),
////
////                        /*
////                        //Retrieve Right Spike Mark
////                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 1...")),
////                        new InstantAction(robot::intakeSamplePos),
////                        strafeWithSubsystems(roadRunnerDrive.pose, yellowSpikeRight.plus(
////                                calculateOffset(180, armExtentionLength))),
////                        new InstantAction(robot::toggleIntake)
////
////                        new InstantAction(() -> this.addTelemetryMessage("Attempting to retrieve Sample...")),
////                        //Drive forward 5 inches to try to "sweep" and pick up the spike mark sample
////                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
////                                .strafeTo(yellowSpikeRight.plus(calculateOffset(180, armExtentionLength-5)))
////                                .build(),
////
////                        new InstantAction(() -> this.addTelemetryMessage("Depositing Sample...")),
////                        new InstantAction(robot::depositSamplePosForward), //Set subsystems to prepare deposit
////                        strafeWithSubsystems(roadRunnerDrive.pose, bucketPosition),
////                        new InstantAction(robot::intakeBackward), //Deposits sample
////                        new SleepAction(0.5), //Wait for intake to completely deposit
////                        */
////
////
////                        //Retrieve Middle Spike Mark
////
////                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 2...")),
////                        new InstantAction(() -> this.addTelemetryMessage("Current Position: "+poseToString(roadRunnerDrive.pose))),
////
////                        roadRunnerDrive.actionBuilder(bucketPosition)
////                                .strafeToLinearHeading(yellowSpikeRightLineUp.position, yellowSpikeRightLineUp.heading)
////                                .stopAndAdd(new InstantAction(robot::intakeSamplePos))
////                                .stopAndAdd(new InstantAction(robot::toggleIntake))
////                                .stopAndAdd(robot.waitForLiftArmPID(2))
////                                .stopAndAdd(new SleepAction(2))
////                                .strafeTo(yellowSpikeRightIntake.position,new TranslationalVelConstraint(1))
////                                .build()
//
//
//
//
//
////                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 2...")),
////                        new InstantAction(() -> this.addTelemetryMessage("Current Position: "+poseToString(roadRunnerDrive.pose))),
////                        strafeWithSubsystems(bucketPosition,yellowSpikeMiddleLineUp),
////
////
////
////
////                        //Evaluate position for a moment (TESTING PURPOSES)
////                        new SleepAction(5),
////
////                        new InstantAction(() -> this.addTelemetryMessage("Current Position: "+poseToString(roadRunnerDrive.pose))),
////                        new InstantAction(() -> this.addTelemetryMessage("Attempting to retrieve Sample...\n" +
////                                "Start Position:" +
////                                poseToString(yellowSpikeMiddleLineUp) +
////                                "\nTarget Position: " +
////                                poseToString(yellowSpikeMiddleIntake)
////                        )),
////                        //Drive forward 5 inches to try to "sweep" and pick up the spike mark sample
//////                        new InstantAction(robot::intakeSamplePos),
//////                        //robot.waitForLiftArmPID(2),
//////                        new InstantAction(robot::toggleIntake),
////
////                        roadRunnerDrive.actionBuilder(yellowSpikeMiddleLineUp)
//////                                .strafeToLinearHeading(yellowSpikeMiddle.plus(calculateOffset(180, armExtentionLength-5)),Math.toRadians(180))
//////                                .lineToY(roadRunnerDrive.pose.position.y - 5)
//////                                .strafeTo(yellowSpikeMiddle.plus(new Vector2d(-5, 0)))
//////                                .strafeTo(yellowSpikeMiddleIntake.position,
//////                                        //Reduce speed a significantly
//////                                        new TranslationalVelConstraint(5))
////                                .lineToX(yellowSpikeMiddleIntake.position.x - 3)
////                                .build(),
////                new InstantAction(() -> this.addTelemetryMessage("Current Position: "+poseToString(roadRunnerDrive.pose)))
//
//                        //Evaluate position for a moment (TESTING PURPOSES)
//                        /*
//                        new SleepAction(5),
//                        new InstantAction(() -> this.addTelemetryMessage("Driving to Bucket...")),
//                        new InstantAction(robot::depositSamplePosForward), //Set new target position
//                        new InstantAction(robot::VerticalArm), //Override Arm Position
//                        strafeWithSubsystems(roadRunnerDrive.pose, bucketPosition),
//                        //Deposit Via Claw
//                        new InstantAction(() -> this.addTelemetryMessage("Deposit Spike 1...")),
//                        new InstantAction(robot::intakeBackward),
//                        new SleepAction(1), //Wait for claw to completely open and deposit
//                        new InstantAction(() -> this.addTelemetryMessage("Stow Arm...")),
//                        new InstantAction(robot::deactivateIntake),
//                        //Move arm temporarily
//                        new InstantAction(robot::VerticalArm),
//                        robot.waitForLiftArmPID(1),
//                        new InstantAction(() -> this.addTelemetryMessage("Stow Lift...")),
//                        new InstantAction(robot::resetLift),
//                        robot.waitForLiftArmPID(3)
//                        */
//
//
//                        /*
//                        new InstantAction(() -> this.addTelemetryMessage("Depositing Sample...")),
//                        new InstantAction(robot::depositSamplePosForward), //Set subsystems to prepare deposit
//                        strafeWithSubsystems(roadRunnerDrive.pose, new Pose2d(bucketPosition,Math.toRadians(225))),
//                        new InstantAction(robot::intakeBackward), //Deposits sample
//                        new SleepAction(0.5), //Wait for intake to completely deposit
//
//
//
//                        //Retrieve Left Spike Mark
//                        new InstantAction(() -> this.addTelemetryMessage("Driving to Spike Mark 3...")),
//                        new InstantAction(robot::intakeSamplePos),
//                        strafeWithSubsystems(roadRunnerDrive.pose, new Pose2d(yellowSpikeLeft.plus(
//                                calculateOffset(180, armExtentionLength)),Math.toRadians(180))),
//                        new InstantAction(robot::toggleIntake),
//
//                        new InstantAction(() -> this.addTelemetryMessage("Attempting to retrieve Sample...")),
//                        //Drive forward 5 inches to try to "sweep" and pick up the spike mark sample
//                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
//                                .strafeTo(yellowSpikeLeft.plus(calculateOffset(180, armExtentionLength-5)))
//                                .build(),
//                        new InstantAction(() -> this.addTelemetryMessage("Depositing Sample...")),
//                        new InstantAction(robot::depositSamplePosForward), //Set subsystems to prepare deposit
//                        strafeWithSubsystems(roadRunnerDrive.pose, new Pose2d(bucketPosition,Math.toRadians(225))),
//                        new InstantAction(robot::intakeBackward), //Deposits sample
//                        new SleepAction(0.5), //Wait for intake to completely deposit
//
//                        //Resets robot's subsystems to original position
//                        new InstantAction(robot::lowerClimb),
//
//                        //Go park in Ascent Zone
//                        roadRunnerDrive.actionBuilder(roadRunnerDrive.pose)
//                                .setTangent(Math.toRadians(90))
//                                .splineTo(new Vector2d(-24,-12),0)
//                                .build()
//                                */
////
////                )
////        ));
//
//
//
//
//
//    }
//
//    public ParallelAction strafeWithSubsystems(Pose2d startPosition, Vector2d endPosition){
//        return strafeWithSubsystems(startPosition, new Pose2d(endPosition,startPosition.heading));
//    }
//
//    public ParallelAction strafeWithSubsystems(Pose2d startPosition, Pose2d endPosition){
//        return new ParallelAction(
//                //Travel to place to deposit
//                roadRunnerDrive.actionBuilder(startPosition)
//                        .strafeToLinearHeading(endPosition.position.plus(
//                                calculateOffset(180, armExtentionLength)
//                        ), endPosition.heading)
//                        .build(),
//                robot.waitForLift(6) //Ensure that subsystems are in the right position
//        );
//    }
//
//    public void addTelemetryMessage(String message){
//        telemetryLines.add(message + " " + timer.time(TimeUnit.SECONDS) + " Elapsed");
//        for (String info: telemetryLines) {
//            telemetry.addLine(info);
//        }
//        telemetry.update();
//    }
//
//    public static Vector2d calculateOffset(double angleDegrees, double distance){
//        return new Vector2d(-distance * Math.cos(Math.toRadians(angleDegrees)), -distance * Math.sin(Math.toRadians(angleDegrees)));
//    }
//
//    public static Pose2d calculateOffset(double angleDegrees, double distance, Pose2d target){
//        Vector2d pos = target.position;
//        return new Pose2d(pos.x - distance * Math.cos(Math.toRadians(angleDegrees)),pos.y - distance * Math.sin(Math.toRadians(angleDegrees)),Math.toRadians(angleDegrees));
//    }
//
//    @SuppressLint("DefaultLocale")
//    public String poseToString(Pose2d pos){
//        return String.format("%f, %f, %f",
//                pos.position.x,
//                pos.position.y,
//                pos.heading.toDouble());
//    }
//
//    public Pose2d duplicatePose(Pose2d pos){
//        return new Pose2d(pos.position, pos.heading);
//    }
//}
