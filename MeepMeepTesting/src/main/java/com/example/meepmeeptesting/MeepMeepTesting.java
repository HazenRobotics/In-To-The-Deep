package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    //Starting Positions
    public static Pose2d startColor = new Pose2d(12,-64,Math.toRadians(90));
    public static Pose2d startColor2 = new Pose2d(12,-64,Math.toRadians(-90));
    public static Pose2d startYellow = new Pose2d(-12, -64, Math.toRadians(90));
    public static Pose2d startYellow2 = new Pose2d(-12, -64, Math.toRadians(90));
    public static Vector2d basketDropOff = new Vector2d(-55, -65);

    public static Vector2d firstSpike = new Vector2d(-49, -26);
    public static Vector2d secondSpike = new Vector2d(-61, -26);
    public static Vector2d thirdSpike = new Vector2d(-70, -26);
    //intake length from center of robot
    public static double intakeLength = 15;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity activePath = colorBot3(meepMeep);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(activePath)
                .start();
    }

    public static Vector2d calculateOffset(double angleDegrees, double distance){
        return new Vector2d(-distance * Math.cos(Math.toRadians(angleDegrees)), -distance * Math.sin(Math.toRadians(angleDegrees)));
    }

    public static RoadRunnerBotEntity Template(MeepMeep meepMeep){
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startColor)
                .build());
        return bot;
    }

    public static RoadRunnerBotEntity colorBot2(MeepMeep meepMeep){
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startColor2)
                //Score Preload
                        .setReversed(true)
                        .splineTo(new Vector2d(0,-36),Math.toRadians(90))
                        .waitSeconds(0.5)
                        .lineToY(-24)
                        .waitSeconds(0.1)
                                .setReversed(false)
                //Go Push Inner Spike
                        .splineTo(new Vector2d(36,-24),Math.toRadians(90))
                        .splineTo(new Vector2d(60,-12),Math.toRadians(0))
                        .setReversed(true)
                                .splineTo(new Vector2d(48,-60),Math.toRadians(-90))
                //Go Push Middle Spike
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(55,-12,Math.toRadians(90)),Math.toRadians(0))
                        .setReversed(true)
                            .splineTo(new Vector2d(55,-60),Math.toRadians(-90))
                //Go Push Outer Spike
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(66,-12,Math.toRadians(90)),Math.toRadians(0))
                        .setReversed(true)
                        .lineToY(-60)

                //Score Specimen 1
                        .setReversed(false)
                        .splineTo(new Vector2d(40,-60),Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .lineToY(-62)
                        .setReversed(true)
                        .splineTo(new Vector2d(0,-36),Math.toRadians(90))
                        .waitSeconds(0.5)
                        .lineToY(-24)

                //Score Specimen 2
                .setReversed(false)
                .splineTo(new Vector2d(40,-60),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-62)
                .setReversed(true)
                .splineTo(new Vector2d(0,-36),Math.toRadians(90))
                .waitSeconds(0.5)
                .lineToY(-24)

                //Score Specimen 3
                .setReversed(false)
                .splineTo(new Vector2d(40,-60),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-62)
                .setReversed(true)
                .splineTo(new Vector2d(0,-36),Math.toRadians(90))
                .waitSeconds(0.5)
                .lineToY(-24)

                .build());
        return bot;
    }

    public static RoadRunnerBotEntity yellowBot3(MeepMeep meepMeep){
        Pose2d bucketDropOff = new Pose2d(-54.5, -59.5, Math.toRadians(225));
        Pose2d innerSampleLineUp = new Pose2d(-24, -34, Math.toRadians(160));
        Pose2d middleSampleLineUp = new Pose2d(-34, -26, Math.toRadians(180));
        Pose2d outerSampleLineUp = new Pose2d(-44, -26, Math.toRadians(180));
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startYellow2)

                //Score preload
                        .strafeToLinearHeading(bucketDropOff.position,bucketDropOff.heading)
                        .waitSeconds(1)

                //Drive to inner Sample
                        .setReversed(true)
                        .splineTo(innerSampleLineUp.position,innerSampleLineUp.heading.toDouble()-Math.toRadians(180))
                        .waitSeconds(1)

                //Score to inner sample
                        .setReversed(false)
                        .lineToX(-36)
//                        .afterDisp()//After traveling some path to pick up a sample, raise lift
                        .splineTo(bucketDropOff.position,bucketDropOff.heading)
                        .waitSeconds(1)

                //Drive to middle sample
                        .setReversed(true)
                        .splineTo(middleSampleLineUp.position,middleSampleLineUp.heading.toDouble()-Math.toRadians(180))
                        .waitSeconds(1)
                        .setReversed(false)

                //Score Middle Sample
                .lineToX(-46)
//                        .afterDisp()//After traveling some path to pick up a sample, raise lift
                .splineTo(bucketDropOff.position,bucketDropOff.heading)

                //Drive to Outer Sample
                        .setReversed(true)
                        .splineTo(outerSampleLineUp.position,outerSampleLineUp.heading.toDouble()-Math.toRadians(180))
                        .waitSeconds(1)
                        .setReversed(false)
                //Score Outer Sample
                .lineToX(-56)
//                        .afterDisp()//After traveling some path to pick up a sample, raise lift
                .splineTo(bucketDropOff.position,bucketDropOff.heading)


                .build());
        return bot;
    }

    public static RoadRunnerBotEntity colorBot3(MeepMeep meepMeep){
        Pose2d bucketDropOff = new Pose2d(50, -50, Math.toRadians(-90));
        Pose2d innerSampleLineUp = new Pose2d(24, -34, Math.toRadians(-160));
        Pose2d middleSampleLineUp = new Pose2d(39, -26, Math.toRadians(-180));
        Pose2d outerSampleLineUp = new Pose2d(49, -26, Math.toRadians(-180));
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startColor2)
            //Score preload
                .setReversed(true)
                .splineTo(new Vector2d(0,-36),Math.toRadians(90))
                .waitSeconds(0.5)
                .lineToY(-24)
                .waitSeconds(0.1)
                .setReversed(false)

            //Drive to inner Sample

                .splineTo(innerSampleLineUp.position,innerSampleLineUp.heading.toDouble()-Math.toRadians(180))
            //Drop off inner Sample to Human Player
                //.afterDisp()//After traveling some path to pick up a sample, raise lift
                .lineToX(36)
                .splineTo(bucketDropOff.position,bucketDropOff.heading)
                .waitSeconds(0.3)

            //Intake middle Sample
                .setReversed(true)
                .splineTo(middleSampleLineUp.position,middleSampleLineUp.heading.toDouble())
                .waitSeconds(0.5)
            //Drop off middle Sample to Human Player
                .setReversed(false)
                //.afterDisp()//After traveling some path to pick up a sample
                .lineToX(46)
                .splineTo(bucketDropOff.position.plus(new Vector2d(10,0)),bucketDropOff.heading)
                .waitSeconds(0.3)

            //Intake Outer Sample
                .setReversed(true)
                .splineTo(outerSampleLineUp.position,outerSampleLineUp.heading.toDouble())
                .waitSeconds(0.5)
            //Drop off Outer Sample to Human Player
                .setReversed(false)
                .lineToX(56)
                //.afterDisp()//After traveling some path to pick up a sample, raise lift
                .splineTo(bucketDropOff.position.plus(new Vector2d(15,0)),bucketDropOff.heading)
                .waitSeconds(0.3)


                //Score Specimens
                //Score Specimen 1
//                .setReversed(true)
                .strafeTo(new Vector2d(40,-60))
                .waitSeconds(0.5)
                .lineToY(-62)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(0,-36),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-24)

                //Score Specimen 2
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(40,-60),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-62)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(0,-36),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-24)

                //Score Specimen 3
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(40,-60),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-62)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(0,-36),Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-24)
                .build());
        return bot;
    }
    public static RoadRunnerBotEntity yellowBot2(MeepMeep meepMeep){
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startYellow2)
                //Score preload sample
                .strafeToLinearHeading(basketDropOff, Math.toRadians(180))
                //At the same time as the spline, raise the lift/arm
                .waitSeconds(2) //Time to deposit

                //Go to first spike
                .strafeTo(firstSpike.plus(calculateOffset(180, intakeLength)))
                //Lower lift/arm at the same time here
                //Activate intake
                .strafeTo(firstSpike.plus(calculateOffset(180, intakeLength-5)))
                .strafeTo(basketDropOff)
                .waitSeconds(2) //Time to deposit
                //Go to second spike
                .strafeTo(secondSpike.plus(calculateOffset(180, intakeLength)))
                //Lower lift/arm at the same time here
                //Activate intake
                .strafeTo(secondSpike.plus(calculateOffset(180, intakeLength-5)))
                .strafeTo(basketDropOff)
                .waitSeconds(2) //Time to deposit
                //Go to third spike

                .strafeTo(thirdSpike.plus(calculateOffset(180, intakeLength)))
                //Lower lift/arm at the same time here
                //Activate intake
                .strafeTo(thirdSpike.plus(calculateOffset(180, intakeLength-5)))
                .strafeToLinearHeading(basketDropOff, Math.toRadians(225))
                .waitSeconds(2) //Time to deposit

                //Park
//                        .turn(Math.toRadians(-90))
//                        .setTangent(Math.toRadians(-45))
////                        .turn(Math.toRadians(-135))
//                        .splineTo(new Vector2d(-55,-40),0)
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-24,-12,Math.toRadians(-90)),Math.toRadians(0))
//                        .strafeToLinearHeading(new Vector2d(-48,-12),Math.toRadians(180))
//                        .strafeToLinearHeading(new Vector2d(-24,-12),Math.toRadians(180))

                .build());
        return bot;
    }

    public static RoadRunnerBotEntity yellowBot(MeepMeep meepMeep){
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startYellow)
                //Hang Preload
                .lineToY(-34)
                .waitSeconds(3) //Hang Specimen
                //Back up to avoid crashing
                .strafeToLinearHeading(new Vector2d(-12,-46),Math.toRadians(180))

                //Pick up new sample
                .strafeTo(new Vector2d(-38,-26))
                //Claw/Arm stuff
                .waitSeconds(3)//Pick up Spike mark

                //Drop off sample in basket
                //Raise lift and arm mid drive
                .strafeTo(basketDropOff)
                .waitSeconds(1)//Drop sample

                //Travel to next Spike Mark
                .strafeTo(new Vector2d(-48,-26))
                //Pick up Spike mark
                .waitSeconds(3)
                .strafeTo(basketDropOff)
                .waitSeconds(1)//Drop Sample

                //Travel to next Spike Mark
                .strafeTo(new Vector2d(-58,-26))
                //Pick up Spike mark
                .waitSeconds(3)
                .strafeTo(basketDropOff)
                .waitSeconds(1)//Drop Sample

                .build());
        return bot;
    }

    public static RoadRunnerBotEntity colorBot(MeepMeep meepMeep){
        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        bot.runAction(bot.getDrive().actionBuilder(startColor)

                //Hang Preload
                .lineToY(-34)
                .waitSeconds(3) //Hang Specimen
                //Back up to avoid crashing
                .strafeToLinearHeading(new Vector2d(12,-46),Math.toRadians(0))

                //Pick up new sample
                .strafeTo(new Vector2d(38,-26))
                //Claw/Arm stuff
                .waitSeconds(3)//Pick up Spike mark


                //Drop of sample for HP
                .strafeTo(new Vector2d(48,-60))
                .waitSeconds(1)//Drop Sample
                .strafeTo(new Vector2d(48,-44))
//                        .turn(Math.toRadians(-90))
                //Retrieve second sample
                .strafeTo(new Vector2d(48,-26))
                .waitSeconds(1)//Pick up new Sample
                .turn(Math.toRadians(-90))

//                .waitSeconds(5)//Place sample in HP zone, wait for HP

                //Pick up specimen from HP
                .strafeTo(new Vector2d(48,-60))
                //Extend/prepare claw here
                .strafeTo(new Vector2d(60,-60))
                //Close claw
                .waitSeconds(1)

                //Drive and hang specimen
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(11, -34),Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(11,-34,Math.toRadians(90 - 1e-6)),Math.toRadians(90))//Travel to hanging bar
                .waitSeconds(3)// Hang Specimen Again

                //Park in ascent zone

                .strafeToLinearHeading(new Vector2d(-36,-36),0)
                .strafeTo(new Vector2d(-36,-12))
                //Rise lift and arm to touch/rest on bar
                .strafeTo(new Vector2d(-24,-12))

                .build());
        return bot;
    }

}