//package org.firstinspires.ftc.teamcode.auto.version2;
//
//import com.acmerobotics.roadrunner.AngularVelConstraint;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.drivetrains.Version2;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.version2.DepositArm;
//import org.firstinspires.ftc.teamcode.subsystems.version2.DepositLift;
//import org.firstinspires.ftc.teamcode.subsystems.version2.ExtendoSlide;
//import org.firstinspires.ftc.teamcode.subsystems.version2.IntakeArm;
//import org.firstinspires.ftc.teamcode.utils.MiscMethods;
//import org.firstinspires.ftc.teamcode.utils.TelemetryRecorder;
//
////@Autonomous(name="Push Specimen Auto")
//public class SpecimenPushAuto extends LinearOpMode {
//
//    Pose2d startPosition = new Pose2d(9,-65,Math.toRadians(90));
//
//    Pose2d inner = new Pose2d(49, -24, 0);
//    Pose2d middle = new Pose2d(59, -24, 0);
//    Pose2d outer = new Pose2d(69, -24, 0);
//
//    double extendoMax = 24;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Version2 robot = new Version2(hardwareMap);
//        MecanumDrive roadRunner = new MecanumDrive(hardwareMap,startPosition);
//        TelemetryRecorder print = new TelemetryRecorder(telemetry);
//
//        double inner = 49, middle = 59, outer = 69;
//        Pose2d innerAnchor = MiscMethods.inverseLerp(new Pose2d(inner,-25,0),29, -120 );
////        Pose2d middleAnchor = MiscMethods.inverseLerp(new Pose2d(middle, -25, 0),24, -135);
////        Pose2d innerLineUp = MiscMethods.inverseLerp(inner, extendoMax,45);
////        Pose2d middleLineUp = MiscMethods.inverseLerp(middle, extendoMax,45);
////        Pose2d outerLineUp = MiscMethods.inverseLerp(outer, extendoMax,45);
//
//        robot.specAutoInit();
//        robot.closeClaw();
//
//        waitForStart();
//        print.resetTimer();
//
//        Actions.runBlocking(new ParallelAction(
//                robot.autoPID(),
//                roadRunner.actionBuilder(startPosition)
//                        .stopAndAdd(print.addInstantMessage("Starting Preload"))
//                        .stopAndAdd(new InstantAction(() ->robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_DEPOSIT_PRELOAD)))
//                        .stopAndAdd(new InstantAction(() -> robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_DEPOSIT_PRELOAD)))
//                        .lineToY(-34)
//                        .stopAndAdd(robot::openClaw)
//                        .stopAndAdd(print.addInstantMessage("Completed Preload"))
//
//                        .stopAndAdd(print.addInstantMessage("Prepare Intake Inner Sample"))
//                        .afterDisp(3, new InstantAction(() -> {
//                            robot.lift.goToPosition(DepositLift.LiftStates.SPECIMEN_INTAKE);
//                            robot.deposit.goToPosition(DepositArm.PivotArmStates.SPECIMEN_INTAKE);
//                            robot.deposit.openClaw();
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.PUSH);}))
//
//
//
//                        .setReversed(true)
//                        .splineToLinearHeading(new Pose2d(innerAnchor.position, innerAnchor.heading.plus(Math.toRadians(180)).toDouble()), Math.toRadians(45))
//
//                        .stopAndAdd(print.addInstantMessage("Intake Inner Sample"))
//                        .stopAndAdd(robot::activateIntake)
//                        .stopAndAdd(new InstantAction(()-> robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND)))
//                        .stopAndAdd(robot.waitForExtendo(2))
//                        .strafeToLinearHeading(innerAnchor.position,Math.toRadians(-35), new AngularVelConstraint(Math.PI))
//                        .stopAndAdd(new InstantAction(()->{
//                            print.addMessage("Deposit Inner");
//                            robot.arm.goToPosition(IntakeArm.IntakeArmStates.HOVER);
//                            robot.reverseIntake();
//                        }))
//
//                        .stopAndAdd(print.addInstantMessage("Intake Middle Sample"))
//                        .strafeToLinearHeading(innerAnchor.position.plus(new Vector2d(10,0)), innerAnchor.heading.plus(Math.toRadians(180)).toDouble())
//                        .stopAndAdd(new InstantAction(()->{
//                                robot.arm.goToPosition(IntakeArm.IntakeArmStates.PUSH);
//                                robot.activateIntake();
//                        }))
//                        .strafeToLinearHeading(innerAnchor.position.plus(new Vector2d(10.1,0.1)), Math.toRadians(-35), new AngularVelConstraint(Math.PI))
////                        .turnTo(Math.toRadians(-30))
//                        .stopAndAdd(print.addInstantMessage("Deposit Middle"))
//                        .stopAndAdd(robot::reverseIntake)
//
//                        .stopAndAdd(print.addInstantMessage("Intake Outer"))
//                        .afterDisp(0.1,new InstantAction(()->robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.HALF_EXTEND)))
//                        .strafeToLinearHeading(innerAnchor.position.plus(new Vector2d(20,0)),innerAnchor.heading.plus(Math.toRadians(180)))
//                        .stopAndAdd(robot::activateIntake)
//                        .stopAndAdd(new InstantAction(()-> robot.extendo.goToPosition(ExtendoSlide.ExtendoStates.FULL_EXTEND)))
//                        .stopAndAdd(robot.waitForExtendo(2,true))
//                        .stopAndAdd(new InstantAction(() -> robot.arm.goToPosition(IntakeArm.IntakeArmStates.TRANSFER)))
//                        .strafeToLinearHeading(innerAnchor.position.plus(new Vector2d(10,-10)),Math.toRadians(-10))
//                        .stopAndAdd(robot::reverseIntake)
//
//
//                        .afterTime(0.3, robot::disableAutoPID)
//                        .build()));
//        print.addMessage("Complete!");
//        while(opModeIsActive()){}
//    }
//
//}
