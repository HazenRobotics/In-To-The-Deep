package org.firstinspires.ftc.teamcode.auto.version2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrains.Version2;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArm;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArmV2;
import org.firstinspires.ftc.teamcode.subsystems.version2.DepositLift;
import org.firstinspires.ftc.teamcode.subsystems.version2.ExtendoSlide;
import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
import org.firstinspires.ftc.teamcode.utils.MiscMethods;
import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;

@Autonomous(name="5 Specimen Auto")
public class SpecimenAuto extends LinearOpMode {

    Pose2d startPosition = new Pose2d(9,-65,Math.toRadians(90));
    double changeX = -1;
    double changeY = -1;
    double inner = 47.5 + changeX, middle = 59 + changeX, outer = 73 + changeX;
    double sampleY = -50 + changeY;

    long extendoWaitTime = 1;
    double ejectorStowDelay = 0.2;
    Pose2d outerLineUp = new Pose2d(middle-5, sampleY+1,Math.toRadians(90));
    Pose2d outerIntake = MiscMethods.lerp(outerLineUp, new Pose2d(outer, -24, 0), 0.1 );

    Pose2d outerIntakeAngled = new Pose2d(outerLineUp.position, outerIntake.heading);

    Pose2d middleIntake = new Pose2d(middle, sampleY + 1, Math.toRadians(90));


    Pose2d innerIntakeAngled = MiscMethods.lerp(middleIntake, new Pose2d(inner, -24, 0), 0.1);

    Pose2d intakePose = new Pose2d(40,-60.5,Math.toRadians(90));

    Pose2d intakeCyclePose = new Pose2d(50, -55, Math.toRadians(-90));

    Pose2d specimenDeposit = new Pose2d(-2,-32,Math.toRadians(90));

    TranslationalVelConstraint depositSpeed = new TranslationalVelConstraint(70);
    double depositGap = 2;

    double ejectAngle = 105;

    double depositDist = -26;

    TelemetryRecorder print;
    Version2 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Version2(hardwareMap);
        MecanumDrive roadRunner = new MecanumDrive(hardwareMap,startPosition);

        print = new TelemetryRecorder(telemetry);


        robot.specAutoInit();
        robot.closeClaw();

        waitForStart();
        print.resetTimer();

        Actions.runBlocking(new ParallelAction(
            robot.autoPID(),
                roadRunner.actionBuilder(startPosition)
                        .stopAndAdd(robot::specimenDeposit)
//                        .stopAndAdd(new InstantAction(() ->robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT_PRELOAD)))
//                        .stopAndAdd(new InstantAction(() -> robot.deposit.goToPosition(DepositArmV2.PivotArmStates.SPECIMEN_DEPOSIT_PRELOAD)))
                        .lineToY(-34)
//                        .stopAndAdd(readLift())
                        .stopAndAdd(robot::openClaw)



                        //Intake Inner Sample
//                        .afterDisp(5,new InstantAction(()-> robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.HALF_EXTEND)))
                        .stopAndAdd(print.addInstantMessage("Begin Inner Sample Intake"))
//                        .afterDisp(3,robot::specimenIntake)
                        .afterDisp(3, new InstantAction(() -> {
                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE);
                            robot.deposit.goToPosition(DepositArmV2.PivotArmStates.SPECIMEN_INTAKE);

                            robot.deposit.openClaw();
                            //drive forward 0.17
//                          robot.arm.goToPosition(IntakeArm.IntakeArmStates.SAMPLE_INTAKE_AUTO);
                           robot.drive(0.17, 0, 0);
                        }))
                        

//                        .afterDisp(3, robot::ejectDown)
//                        .afterDisp(3.1, new InstantAction(()->robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_INTAKE_AUTO)))
//                        .afterTime(3.1, new InstantAction(() -> robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_AUTO)))
//                        .afterDisp(3.2,new InstantAction(() -> robot.arm.goToPosition(IntakeArm.IntakeArmStates.SAMPLE_INTAKE_AUTO)))

                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(inner,sampleY),Math.toRadians(90))
                        .stopAndAdd(robot::activateIntake)
                        .stopAndAdd(new InstantAction(()->{
                            print.addMessage("Extending Extendo");
                            //Comment out line below and move forward
                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND);
                        }))
                        .stopAndAdd(robot.waitForExtendo(extendoWaitTime, true))
                        .stopAndAdd(new InstantAction(()-> {
                            print.addMessage("Retract Intake");
                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
                            //Comment out line below and move forward
                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);
                        }))
                        .turnTo(Math.toRadians(ejectAngle))
                        .afterTime(extendoWaitTime/2.0,robot::deactivateIntake)
                        .stopAndAdd(robot.waitForExtendo(extendoWaitTime))
                        .waitSeconds(0.2)
                        .stopAndAdd(print.addInstantMessage("Eject Sample"))
                        .stopAndAdd(robot::ejectUp) //Deposit inner sample
                        .waitSeconds(ejectorStowDelay)
                        .stopAndAdd(robot::ejectDown)

//                        .waitSeconds(3)
                        //Intake Middle Sample
                        .stopAndAdd(print.addInstantMessage("Begin Middle Sample Intake"))
                        .afterTime(ejectorStowDelay, robot::reverseIntake)
                        .afterTime(0.25,new InstantAction(() -> robot.arm.goToPosition(IntakeArm.IntakeArmStates.SAMPLE_INTAKE_AUTO)))
                        .strafeToLinearHeading(new Vector2d(middle, sampleY + 1), Math.toRadians(90))
                        .stopAndAdd(robot::activateIntake)
                        .stopAndAdd(new InstantAction(()-> robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND)))
                        .stopAndAdd(robot.waitForExtendo(extendoWaitTime, true))
                        .stopAndAdd(print.addInstantMessage("Retract Intake"))
                        .stopAndAdd(new InstantAction(()-> {
                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);}))

                        .turnTo(Math.toRadians(ejectAngle-5))
                        .afterTime(extendoWaitTime/2.0,robot::deactivateIntake)
                        .stopAndAdd(robot.waitForExtendo(extendoWaitTime))
                        .waitSeconds(0.2)
//                        .stopAndAdd(robot::deactivateIntake)
                        .stopAndAdd(robot::ejectUp)//Deposit Middle Sample
                        .stopAndAdd(print.addInstantMessage("Eject Sample"))


                        //Intake Outer Sample
                        .stopAndAdd(print.addInstantMessage("Begin Outer Sample Intake"))
                        .afterTime(ejectorStowDelay,robot::reverseIntake)
                        .afterTime(0.15,new InstantAction(() -> robot.arm.goToPosition(IntakeArm.IntakeArmStates.SAMPLE_INTAKE_AUTO)))
                        .waitSeconds(0.3)
                        .strafeToLinearHeading(outerLineUp.position,outerIntake.heading.plus(Math.toRadians(-185)))
                        .stopAndAdd(robot::activateIntake)
                        .stopAndAdd(new InstantAction(()-> robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND)))
                        .stopAndAdd(robot.waitForExtendo(extendoWaitTime,true))
                        .stopAndAdd(print.addInstantMessage("Retract Intake"))
                        .strafeToLinearHeading(new Vector2d(outerLineUp.position.x,-55), intakePose.heading)
                        .stopAndAdd(new InstantAction(()-> {
                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER);
                            robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.TRANSFER);}))
                        .turnTo(Math.toRadians(ejectAngle))
//                        .turnTo(90)
                        .afterTime(extendoWaitTime/2.0,robot::deactivateIntake)
                        .stopAndAdd(robot.waitForExtendo(extendoWaitTime))
                        .waitSeconds(0.2)
                        .stopAndAdd(print.addInstantMessage("Eject Sample"))
//                        .stopAndAdd(robot::deactivateIntake)
                        .stopAndAdd(robot::ejectUp) //Deposit Outer Sample

                        .waitSeconds(ejectorStowDelay)
                        .stopAndAdd(robot::ejectDown)
//                        .waitSeconds(ejectorStowDelay + 0.1)






//                        .strafeToLinearHeading(new Vector2d(48,-48),Math.toRadians(90))
                        .stopAndAdd(robot::reverseIntake)
                        .stopAndAdd(new InstantAction(()-> {
                            robot.specimenIntake();
//                            robot.deposit.goToPosition(DepositArmV2.PivotArmStates.SPECIMEN_INTAKE_SIDEWAYS);
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_SIDEWAYS);
                            robot.deposit.openClawAuto();}))
                        .afterTime(1, robot::deactivateIntake)
//                        .waitSeconds(2)

//                        //Intake Spec 1
                        .stopAndAdd(print.addInstantMessage("Specimen Intake 1"))
                        .setReversed(true)
//                        .lineToY(intakePose.position.y)
                        .strafeToLinearHeading(new Vector2d(outerLineUp.position.x ,intakePose.position.y - 1),intakePose.heading)
                        .stopAndAdd(robot::closeClaw)
                        .waitSeconds(0.05)
                        .stopAndAdd(print.addInstantMessage("Specimen Deposit 1"))
//                        .stopAndAdd(new InstantAction(()-> robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_RAISE)))
                        .afterDisp(1.5,new InstantAction(() -> {
                            robot.specimenDeposit();
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_DEPOSIT_SIDEWAYS);
                        }))
                        .setReversed(false)
                        .splineTo(specimenDeposit.position, specimenDeposit.heading, depositSpeed)
                        .lineToY(depositDist, depositSpeed)
                        .stopAndAdd(robot::openClaw) //Deposit Spec1
//                        .stopAndAdd(readLift())
//
                        //Intake Spec 2
                        .stopAndAdd(print.addInstantMessage("Specimen Intake 2"))
                        .afterDisp(1, new InstantAction(()-> {
                            robot.specimenIntake();
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_INTAKE_SIDEWAYS);
                            robot.deposit.openClawAuto();}))
                        .setReversed(true)
                        .strafeTo(intakePose.position.plus(new Vector2d(0,0.3)))
//                        .splineTo(intakeCyclePose.position,intakeCyclePose.heading)
//                        .lineToY(intakePose.position.y)
                        .stopAndAdd(robot::closeClaw)
                        .waitSeconds(0.05)
                        .stopAndAdd(print.addInstantMessage("Specimen Deposit 2"))
                        .stopAndAdd(new InstantAction(()-> robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_RAISE)))
                        .afterDisp(1.5,new InstantAction(() -> {
                            robot.specimenDeposit();
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_DEPOSIT_SIDEWAYS);
                        }))
                        .setReversed(false)
                        .splineTo(specimenDeposit.position.plus(new Vector2d(depositGap,0)),specimenDeposit.heading, depositSpeed)
                        .lineToY(depositDist, depositSpeed)
                        .stopAndAdd(robot::openClaw)
//                        .stopAndAdd(readLift())

                        //Intake Spec 3
                        .stopAndAdd(print.addInstantMessage("Specimen Intake 2"))
                        .afterDisp(1, new InstantAction(()-> {
                            robot.specimenIntake();
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_INTAKE_SIDEWAYS);
                            robot.deposit.openClawAuto();}))
                        .setReversed(true)
                        .strafeTo(intakePose.position.plus(new Vector2d(0,0.3)))
//                        .splineTo(intakeCyclePose.position,intakeCyclePose.heading)
//                        .lineToY(intakePose.position.y)
                        .stopAndAdd(robot::closeClaw)
                        .waitSeconds(0.05)
                        .stopAndAdd(print.addInstantMessage("Specimen Deposit 2"))
                        .stopAndAdd(new InstantAction(()-> robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_RAISE)))
                        .afterDisp(1.5,new InstantAction(() -> {
                            robot.specimenDeposit();
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_DEPOSIT_SIDEWAYS);
                        }))
                        .setReversed(false)
                        .splineTo(specimenDeposit.position.plus(new Vector2d(depositGap * 2,0)),specimenDeposit.heading, depositSpeed)
                        .lineToY(depositDist, depositSpeed)
                        .stopAndAdd(robot::openClaw)
//                        .stopAndAdd(readLift())

                        //Out of time, go Park
                        .afterDisp(2,robot::transferPosition)
                        .strafeTo(intakePose.position)


                        //Intake Spec 4
//                        .stopAndAdd(print.addInstantMessage("Specimen Intake 2"))
//                        .afterDisp(1, new InstantAction(()-> {
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_INTAKE_SIDEWAYS);
//                            robot.deposit.openClawAuto();}))
//                        .setReversed(true)
//                        .strafeTo(intakePose.position)
////                        .splineTo(intakeCyclePose.position,intakeCyclePose.heading)
////                        .lineToY(intakePose.position.y)
//                        .stopAndAdd(robot::closeClaw)
//                        .waitSeconds(0.05)
//                        .stopAndAdd(print.addInstantMessage("Specimen Deposit 2"))
//                        .stopAndAdd(new InstantAction(()-> robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE_RAISE)))
//                        .afterDisp(1.5,new InstantAction(() -> {
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT_SIDEWAYS);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_DEPOSIT_SIDEWAYS);
//                        }))
//                        .setReversed(false)
//                        .splineTo(specimenDeposit.position.plus(new Vector2d(depositGap * 3,0)),specimenDeposit.heading, depositSpeed)
//                        .lineToY(depositDist, depositSpeed)
//                        .stopAndAdd(robot::openClaw)
//                        .stopAndAdd(readLift())




                        .stopAndAdd(new InstantAction(() -> robot.deposit.goToPosition(DepositArmV2.PivotArmStates.TRASNFER)))
                        .stopAndAdd(robot::disableAutoPID)
                        .build()
        ));

        print.addMessage("Completed!");
        while (opModeIsActive()){}
    }

    public Action readLift(){
        return new ReadLift();
    }

    class ReadLift implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            print.addMessage("Lift Current Position: " + robot.lift.getPosition());
            print.addMessage("Lift Target Position: " + robot.lift.getTarget());
            return false;
        }
    }
}
