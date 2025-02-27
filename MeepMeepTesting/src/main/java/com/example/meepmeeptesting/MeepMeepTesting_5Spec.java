package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_5Spec {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .setDimensions(14,14)
                .build();
        Pose2d beginPose = new Pose2d(7.5, -63, Math.toRadians(-90));     //start at wall starting

        VelConstraint velSlow = new TranslationalVelConstraint(30);
        VelConstraint velMedium = new TranslationalVelConstraint(60);
        VelConstraint velFast = new TranslationalVelConstraint(70);
        AccelConstraint accSlow = new ProfileAccelConstraint(-30,30);
        AccelConstraint accMedium = new ProfileAccelConstraint(-60,60);
        AccelConstraint accFast = new ProfileAccelConstraint(-70,70);

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(beginPose)
                /** wall to Submerge-score preload */
                .setReversed(true)
                .lineToYConstantHeading(-36.5,velFast, accMedium)          //to submersible pole to preload score
                .waitSeconds(0.1)

                /** 1st move back from submersible pole--BY SPLINE THEN STRAFE to push Sample 4,5,6 Home*/
                        .setReversed(false)
                .splineToLinearHeading(new Pose2d(34,-45, Math.toRadians(-90)),         //pushSample6Home, midway by Spline
                        Math.toRadians(60), velFast, accMedium)

                .setReversed(true)
                .strafeTo(new Vector2d(43,-7), velFast, accMedium)    //strafe from midway to Sample4

                .strafeTo(new Vector2d(47, -50), velFast, accFast)    //pushSample4Home

                .setReversed(true)
                .strafeTo(new Vector2d(54, -8), velFast, accFast)     //toward Sample5
                .strafeTo(new Vector2d(57, -50), velFast, accMedium)  //pushSample5Home

//                .setReversed(true)
//                .strafeTo(new Vector2d(62, -8), velFast, accMedium)   //toward Sample6

                /** push sample 6 home and goto Wall */
                        .setReversed(false)
//                .splineToLinearHeading(new Pose2d(62,-35, Math.toRadians(-90)),         //pushSample6Home, midway by Spline
//                        Math.toRadians(-90), velFast, accMedium)
//                .splineToLinearHeading(new Pose2d(40,-58, Math.toRadians(-90)), Math.toRadians(-90), velFast, accMedium)       //pushSample6Home, midway by Spline

                .splineToLinearHeading(new Pose2d(40,-63.5, Math.toRadians(-90)),       //pushSample6Home, by Spline to Wall
                        Math.toRadians(-90), velFast, accMedium)
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"

                .waitSeconds(0.5)



//-----------cycle 1 to score Specimen1
                /** already Spec1; now score Spec1 */
                /********************TO BE TESTED Score Spec1 */
                .setReversed(true)              //need this for both test cases as robot is moving backward
        //        .splineToConstantHeading(new Vector2d(38,-61.5),Math.toRadians(135),
          //              velFast, accMedium)                                                       //more more diagonal path to Submersible

                //.splineToConstantHeading(new Vector2d(3,-36),Math.toRadians(90))  //spline to submersible
//                .splineToConstantHeading(new Vector2d(3,-35.5),Math.toRadians(90), velFast, accMedium)  //straighten out robot with another spline to submersible
//                .splineToConstantHeading(new Vector2d(3,-35),Math.toRadians(90), velFast, accMedium)    //straighten out robot with another spline to submersible
                // scored specimen X = 7.5 (preload), now 3, -1, -5, -9)

                .splineToLinearHeading(new Pose2d(38, -61.5, Math.toRadians(-90)), Math.toRadians(135), velFast, accMedium) //splineToConstantHeading(new Vector2d(38,-61.5),Math.toRadians(135),
                //more more diagonal path to Submersible
                //.splineToConstantHeading(new Vector2d(3,-36),Math.toRadians(90))      //COMMENTING OUT TO TEST more diagonal path to Score Spec1
                .splineToLinearHeading(new Pose2d(3, -35.5, Math.toRadians(-90)), Math.toRadians(90), velFast, accMedium)  //splineToConstantHeading(new Vector2d(3,-35.5),Math.toRadians(90), velFast, accMedium)      //straighten out robot with another spline to submersible
                .splineToLinearHeading(new Pose2d(3, -35, Math.toRadians(-90)), Math.toRadians(90), velFast, accMedium)

                .waitSeconds(1)         //open claw at submersible


                /** Collect Spec2 */
                /**********************TO BE TESTED*/
                .strafeToLinearHeading(new Vector2d(40,-63.5),
                        Math.toRadians(-90), velFast, accMedium)                        //1st move back from submersible pole; **test direct strafe, not spline
                /** Comment below out to test */
//                .strafeToLinearHeading(new Vector2d(3,-40),
//                        Math.toRadians(-90), velFast, accMedium)                        //1st move back from submersible pole
//                .splineToConstantHeading(new Vector2d(40,-62.5),Math.toRadians(-90))  //spline to more to near wall
//                .splineToConstantHeading(new Vector2d(40,-63),Math.toRadians(-90))  //spline to more to near wall
//                .splineToConstantHeading(new Vector2d(40,-64.5),Math.toRadians(-90))  //spline to more to near wall
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"


//-----------cycle 2 to score Specimen2
                /** Score Spec2 */
                .setReversed(true)                                                          //need this as robot is moving backward
                .splineToConstantHeading(new Vector2d(-1,-36),Math.toRadians(90))     // scored specimen X = 7.5 (preload), 3, now -1, -9, -9)
                .splineToConstantHeading(new Vector2d(-1,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-1,-35),Math.toRadians(90))

                /** Collect Spec3 */
                .strafeToLinearHeading(new Vector2d(-1,-40),
                    Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(40,-62.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-63),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-63.5),Math.toRadians(-90))
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"

//-----------cycle 3 to score Specimen3
                /** Score Spec3 */
                .setReversed(true)                                                          //need this as robot is moving backward
                .splineToConstantHeading(new Vector2d(-5,-36),Math.toRadians(90))   // scored specimen X = 7.5 (preload), 3, -1, now -5, -9)
                .splineToConstantHeading(new Vector2d(-5,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-5,-35),Math.toRadians(90))

                /** Collect Spec4 */
                .strafeToLinearHeading(new Vector2d(-5,-40),
                    Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(40,-62.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-63),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-63.5),Math.toRadians(-90))
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"

//-----------cycle 4 to score Specimen4
                /** Score Spec4 */
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-9,-36),Math.toRadians(90))   // scored specimen X = 7.5 (preload), 3, -1, -5, now -9)
                .splineToConstantHeading(new Vector2d(-9,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-9,-35),Math.toRadians(90))


/** Park Home */
                .strafeToLinearHeading(new Vector2d(40,-63.5),
                        Math.toRadians(-90), velFast, accFast)  //1st move back from submersible pole
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"

                .build());



        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(30, 30, Math.toRadians(180)))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                //.addEntity(mySecondBot)
                .start();
    }
}