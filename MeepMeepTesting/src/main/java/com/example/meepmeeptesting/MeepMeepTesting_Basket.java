package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_Basket {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .setDimensions(14,14)
                .build();
    //    Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case
        Pose2d beginPose_robot2 = new Pose2d(-16, -62, Math.toRadians(-90));

        Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case

//        int debugLevel = 499;
//        Telemetry telemetryA;
//        Timer pathTimer, actionTimer, opmodeTimer;
//        Pose2d pose;
//
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

//        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
//        //autoDebug(500, "Auto:Init", "DONE");
//        telemetryA.update();

        //adjust these settings as needed for use in the trajectory codes
        VelConstraint velSlow = new TranslationalVelConstraint(15);
        VelConstraint velFast = new TranslationalVelConstraint(45);
        VelConstraint velPark = new TranslationalVelConstraint(60);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15, 15);
        AccelConstraint accFast = new ProfileAccelConstraint(-45, 45);
        AccelConstraint accPark1 = new ProfileAccelConstraint(-60, 60);
        AccelConstraint accPark2 = new ProfileAccelConstraint(-80, 80);



        double readyPosition = 0.43;
        double grabPosition = 0.34;
        double intakeAxonPosition = 0.635;
        double sweeperIn = 0.09;
        double sweeperOUT = 0.5;
        double intakeDown = 0.96;

        Vector2d grabPosePosition1 = new Vector2d(-59, -48);
        Vector2d grabPosePosition3 = new Vector2d(-65, -52.5);
        Vector2d BasketDropPosition = new Vector2d(-55.5, -52.5);

        /**  .lineToX(24.5,velSlow,accSlow)        //example on how to use these in drive.actionBuilder */

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(beginPose)
                .setTangent(45)
                .strafeToSplineHeading(new Vector2d(-57.5, -52.5), Math.toRadians(45))
                .waitSeconds(0.1)

   //     TrajectoryActionBuilder grabPose = preScore.endTrajectory().fresh()
                .strafeToSplineHeading(grabPosePosition1, Math.toRadians(83))

     //   TrajectoryActionBuilder turnToBasket3 = grabPose.endTrajectory().fresh()
                .waitSeconds(0.75)
                .strafeToSplineHeading(new Vector2d(-57.25, -52.5), Math.toRadians(45.5))

       // TrajectoryActionBuilder grabPose2 = turnToBasket3.endTrajectory().fresh()
                .turnTo(Math.toRadians(109.43))
//                .strafeToSplineHeading(grabPosePosition, Math.toRadians(97));

      //  TrajectoryActionBuilder turnToBasket20 = grabPose2.endTrajectory().fresh()
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(46))

     //   TrajectoryActionBuilder grabPose3 = turnToBasket3.endTrajectory().fresh()
                .strafeToSplineHeading(grabPosePosition3, Math.toRadians(119.40))
//                .strafeToSplineHeading(grabPosePosition, Math.toRadians(124));
    //    TrajectoryActionBuilder move = grabPose3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-63, -47.5), Math.toRadians(109))
//                .strafeToSplineHeading(grabPosePosition, Math.toRadians(124));

      //  TrajectoryActionBuilder turnToBasket10 = move.endTrajectory().fresh()
                .waitSeconds(0.65)
                .strafeToSplineHeading(new Vector2d(-57.5, -52.5), Math.toRadians(45))

    //    TrajectoryActionBuilder submersible = turnToBasket3.endTrajectory().fresh()
              //  .setTangent(45)
                .splineToSplineHeading(new Pose2d(-23.25, 0, 0), Math.toRadians(35), velFast, accFast)

    //    TrajectoryActionBuilder turnToBasket2 = submersible.endTrajectory().fresh()
              //  .setTangent(45)
                .strafeToConstantHeading(new Vector2d(-25, 0), velFast, accFast)
                .strafeToSplineHeading(new Vector2d(-59.5, -53), Math.toRadians(45.5), velFast, accFast)

     //   TrajectoryActionBuilder submersiblePark = turnToBasket3.endTrajectory().fresh()
              //  .setTangent(45)

          //      .strafeToLinearHeading(new Vector2d(-35, -15), Math.toRadians(180), velPark, accPark1)
                .strafeToLinearHeading(new Vector2d(-22, 0), Math.toRadians(180), velPark, accPark2)
//                .strafeToSplineHeading(new Vector2d(-35, -15), Math.toRadians(180), velFast, accFast)
//                .strafeToSplineHeading(new Vector2d(-22, -15), Math.toRadians(180), velFast, accPark)

                .build());






        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .setDimensions(14,14)
                .build();


        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(beginPose_robot2)
                .strafeToConstantHeading(new Vector2d(15, -63),velSlow , accSlow)
                .strafeToConstantHeading(new Vector2d(-16, -63),velSlow , accSlow)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .addEntity(mySecondBot)
                .start();
    }
}