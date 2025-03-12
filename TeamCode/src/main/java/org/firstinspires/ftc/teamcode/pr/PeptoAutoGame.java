//package org.firstinspires.ftc.teamcode.pr;
//
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drivetrains.Mecanum;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//
////@Autonomous(name ="A Pepto Auto Challenege")
//
//
//public class PeptoAutoGame extends LinearOpMode
//{
//    private Mecanum robot;
//    private Pose2d startPosition = new Pose2d(0,0,Math.toRadians(0));
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new Mecanum(hardwareMap);
//
//        waitForStart();
//        Actions.runBlocking(new ParallelAction(
//                robot.actionBuilder(startPosition)
//                .stopAndAdd(()->robot.drive(1, 0, 0))
//                .build())
//        ));
//    }
//}
