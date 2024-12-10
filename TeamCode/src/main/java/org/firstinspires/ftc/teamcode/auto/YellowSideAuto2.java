package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

@Autonomous(name="YellowSideAuto2")
public class YellowSideAuto2 extends LinearOpMode {


        public boolean debugging = true;

        public boolean[] enabledSections = {
                true,
                true,
                false
        };

        //Key Points of Interest
//        public Pose2d startPose = new Pose2d(-7.5,-60,Math.toRadians(-90));
        public Pose2d startPose = new Pose2d(-7.5,-64.25,Math.toRadians(90));
        public Pose2d submersibleBar = new Pose2d(0,-31,Math.toRadians(-90));
        public Pose2d specimenScorePose = new Pose2d(0,-24,Math.toRadians(-90));
        public Pose2d specimenScoreLineUp = new Pose2d(0,-36,Math.toRadians(-90));
        public Pose2d sampleScorePoseForward = new Pose2d(-54,-54,Math.toRadians(225));

        //Locations relative to the Submersible

    //Sample Intake Positions
    public Pose2d innerSampleIntake, middleSampleIntake, outerSampleIntake;
    //Sample Line Up Positions
    public Pose2d innerSampleLineUp, middleSampleLineUp, outerSampleLineUp;

    public double armIntakeLength = 17;

    public void initSamplePositions(double y_sample_actual){
        //Sample Intake Positions
        innerSampleIntake = new Pose2d(-49,y_sample_actual,Math.toRadians(160));
        middleSampleIntake = new Pose2d(-59,y_sample_actual,Math.toRadians(180));
        outerSampleIntake = new Pose2d(-69,y_sample_actual,Math.toRadians(180));

        //Sample Line Up Positions
        innerSampleLineUp = new Pose2d(-24,y_sample_actual-9.5,Math.toRadians(160));
        middleSampleLineUp = new Pose2d(0,y_sample_actual,0);
        outerSampleLineUp = new Pose2d(0,y_sample_actual,0);
    }

    double submersibleBarPos = -22; // default position, changes later on

    public double x_error_offset = 0;
    public double y_error_offset = 0;
    public double heading_error_offset = 0;

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
        robot.initializePowerStates();

        //Score Preload
        if (enabledSections[0]){
            //Raises Lift to began the distance check
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    new SequentialAction(
                            new InstantAction(robot::depositSpecimenPosForward),
                            new SleepAction(0.5),
                            robot.endPID(),
                            telemetryRecorder.addIntantMessage("Ended Initial Check")
                    )
            ));
            //Check the distance, take 10 samples to filter out faulty data
            //Default assumption for distance to bar is 42
            double distanceCheck = distanceCheck(42,35,45,100);
            //Should be around -22 for BarPos, -30 if calculating the bot's position
            submersibleBarPos = startPose.position.y + distanceCheck;
            telemetryRecorder.addMessage("Sub recorded position: " + submersibleBarPos);
            //Samples are approximately 4 inches away from the submersible in the y-direction
            initSamplePositions(submersibleBarPos - 4);
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
            distanceCheck = distanceCheck(8,4,12,100);
            submersibleBarPos = roadRunner.pose.position.y + distanceCheck;
            telemetryRecorder.addMessage("Sub recorded position: " + submersibleBarPos);
            telemetryRecorder.addMessage("Preload Scoring Complete!");
        }
        //Intake Sample
        if (enabledSections[1]){
            telemetryRecorder.addMessage("Beginning Inner Sample Intake");
            telemetryRecorder.addMessage("Calculated y_coordinate to be: " + innerSampleIntake.position.y);
            Actions.runBlocking(new ParallelAction(
                    robot.autoPID(),
                    roadRunner.actionBuilder(roadRunner.pose)
//                            .afterDisp(8, new InstantAction(robot::intakeSamplePos))
                            .stopAndAdd(new InstantAction(robot::VerticalArm))
                            .lineToY(submersibleBarPos - 16)
                            .strafeToLinearHeading(innerSampleLineUp.position,Math.toRadians(160))
                            .stopAndAdd(new InstantAction(robot::intakeSamplePos))
                            .afterTime(0.1,new InstantAction(robot::toggleIntake))
                            .stopAndAdd(robot.waitForLiftArmPID(2))
                            .waitSeconds(0.25)

                            .lineToY(1+ innerSampleIntake.position.y-(armIntakeLength*Math.sin(Math.toRadians(160))),
                                    new TranslationalVelConstraint(7.5))
                            .afterTime(0, new InstantAction(robot::VerticalArm))
                            .afterTime(0.5, new InstantAction(() ->robot.lift.goToPosition(Lift.LiftStates.SAMPLE_DEPOSIT)))

                            .splineTo(sampleScorePoseForward.position,sampleScorePoseForward.heading)
                            .stopAndAdd(robot.waitForLiftArmPID(4))
                            .stopAndAdd(telemetryRecorder.addIntantMessage("Ready to deposit!"))
                            .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(Arm.ArmState.SAMPLE_DEPOSIT_FORWARD)))
                            .waitSeconds(0.2)
                            .stopAndAdd(robot::intakeBackward)
                            .waitSeconds(0.1)
                            .stopAndAdd(robot::intakeStop)
                            .stopAndAdd(robot.endPID())
                            .build()
            ));
        }
        //Intake Sample
        if (enabledSections[2]){
            telemetryRecorder.addMessage("Beginning Inner Sample Deposit");
        }

        //Keeps the OpMode running to observe telemetry data.
        while (debugging && opModeIsActive()){}
    }

    public void evaluateError(Pose2d target){
        x_error_offset += target.position.x - roadRunner.pose.position.x;
        y_error_offset += target.position.y - roadRunner.pose.position.y;
        heading_error_offset += target.heading.minus(roadRunner.pose.heading);
    }

    public InstantAction evaluateErrorInstant(Pose2d target){
        return new InstantAction(() -> evaluateError(target));
    }

    public Pose2d addOffset(Pose2d target){
        return new Pose2d(target.position.x + x_error_offset,
                target.position.y + y_error_offset,
                target.heading.toDouble() + heading_error_offset);
    }

    public void setScoreBar(){
        Pose2d botData = roadRunner.pose;
        submersibleBar = new Pose2d(
                0,
                botData.position.y - 7,
                botData.heading.toDouble()
        );
    }

    public InstantAction setScoreBarInstant(){
        return new InstantAction(this::setScoreBar);
    }

    public Pose2d addPose2d(Pose2d pos1, Pose2d pos2){
        return new Pose2d(
                pos1.position.x + pos2.position.x,
                pos1.position.y + pos2.position.y,
                pos1.heading.toDouble() + pos2.heading.toDouble()
        );
    }

    @SuppressLint("DefaultLocale")
    public String errorToString(){
        return String.format("X Error: %f.2\n" +
                "Y Error: %f.2\n" +
                "Heading Error: %f.2\n",
                x_error_offset,
                y_error_offset,
                heading_error_offset);
    }


    public double distanceCheck(double defaultVal, double minVal, double maxVal, double sampleAttempts){
        int numOfAcceptableVals = 0;
        double totalVal = 0;
        for(int i=0; i<sampleAttempts; i++) {
            double tempCheck = distanceSensor.getDistanceInches();
            //Acceptable range for distance sensor
            if ( minVal < tempCheck && tempCheck < maxVal){
                totalVal += tempCheck;
                numOfAcceptableVals++;
            }
        }
        if (numOfAcceptableVals < (sampleAttempts/10)){
            return totalVal / numOfAcceptableVals;
        }
        return defaultVal;
    }
}
