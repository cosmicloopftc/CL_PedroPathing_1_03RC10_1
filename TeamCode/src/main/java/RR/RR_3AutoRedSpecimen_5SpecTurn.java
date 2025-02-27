package RR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

import Hardware.HardwareNoDriveTrainRobot;

/**2/1/2025:  add parallel action--3 specimen scores and then head home.
 *2/5/2025:     create variables for servos positions at wall and scoring and use in later codes
 *              add AUTOStorageConstant to end to transfer to Teleop
 *              use strateTo from wall to submersible for first specimen
 *
 *2/9/2025:   combine push Sample home into 1 trajectory
 *2/9to10/2025: use .afterTime method--able to complete 1+3 specimen and park
 * TODO: check location to pole to be able to consistently score/place specimen on pole
 */


@Config
@Autonomous(name = "RR_3AutoRedSpecimen_Turn_5Spec v1.1", group = "Auto")
public class RR_3AutoRedSpecimen_5SpecTurn extends LinearOpMode {


    //TODO: setup initial position for all subsystems
    //    public static double autoEnd_SliderMotorPosition,
    //            autoEnd_Slider_ServoArmPosition;


    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init





    //-------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode() throws InterruptedException {
        double sliderPower = 0.5;

        AutoOuttakeSliderAction autoOuttakeSliderAction = null;
        autoRobot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class
        autoRobot.imu.resetYaw();

        String AllianceBasketOrSpecimen = "1RedSpecimen";
        //       Pose2d beginPose = new Pose2d(38, -63, Math.toRadians(-90));    //TODO: would overide this for each case
        Pose2d beginPose = new Pose2d(7.5, -63, Math.toRadians(-90));
        Pose2d collectPose = new Pose2d(40, -64.5, Math.toRadians(-90));

        int debugLevel = 499;
        Telemetry telemetryA;
        Timer pathTimer, actionTimer, opmodeTimer;
        Pose2d pose;

        opmodeTimer = new Timer();
        //opmodeTimer.resetTimer();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //autoDebug(500, "Auto:Init", "DONE");
        telemetryA.update();

        //adjust these settings as needed for use in the trajectory codes
        VelConstraint velSlow = new TranslationalVelConstraint(15);
        VelConstraint velMedium = new TranslationalVelConstraint(60);
        VelConstraint velFast = new TranslationalVelConstraint(90);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15,15);
        AccelConstraint accMedium = new ProfileAccelConstraint(-60,60);
        AccelConstraint accFast = new ProfileAccelConstraint(-90,90);


        //TODO: SERVO AND SLIDER POSITIONS---confirm positions
        double clawClose_Pos = 0.30; // change from Eduardo 2/17  1.0;     //claw close
        double clawClose_pauseTimeSecond = 0.3;         //hold time to allow time for claw to close and specimen to adjust position
        double start_OuttakeArmAxonPos =  0.28;         //position inside robot at start

        double scoring_OuttakeArmAxon_ScoringPos = 0.9;       //rotate outtake arm to scoring position
        int scoring_OutakeSlider_ScoringPos = 1330;              //raise outtake  slider to scoring position
        double scoring_OuttakeExtension_ScoringPos =  1;   //extend outtake out to scoring position
        double clawOpen_Pos = 0.0;  // change from Eduardo 2/17  0.7;       //open claw after score
        double clawOpen_pauseTimeSecond = 0.2;

        double wallPickup_OuttakeArmAxonPos = 0.28;         //rotate outtake arm into robot to wall pickup position
        int wallPickup_SliderPos = 0;                       //outtake slider position to pickup specimen from wall
        double wallPickup_OuttakeExtensionPos =  0.85;      //extend out arm out (need to place this to the place after preloadscore)


        /** START */
        TrajectoryActionBuilder preloadScore;
        preloadScore = drive.actionBuilder(beginPose)
                .setReversed(true)
                .lineToYConstantHeading(-35,velMedium, accMedium);             //start and move to submersible pole to score

        /** COMBINE MOVES OF PUSHING SPECIMEN 4 AND 5 AND 6 HOME */
        TrajectoryActionBuilder combine_preloadMove_BackPush2SpecToHome_CollectSpec1 = preloadScore.endTrajectory().fresh()
                .setReversed(false)
                .stopAndAdd(new AutoClawAction(clawOpen_Pos, clawOpen_pauseTimeSecond))        //open claw
                .afterTime(0.5, autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0))            //TODO: rotate outtake arm to ready position to grab specimen on wall
                .afterTime(0.5, autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos))                       //TODO: extend out arm out (need to this to the place after preloadscore
                .afterTime(0.5, autoOuttakeSliderAction(wallPickup_SliderPos, 1))                      //TODO: ?outtake slider position to pickup specimen from wall
                .afterTime(0.3, autoIntakeSliderAction(290,1,0))
                .afterTime(0.5, autoIntakeSpiner(1, 0.5))
//                .afterTime(1.3, autoIntakeServoAxonAction(0.97))
//                .afterTime(3.1, autoIntakeServoAxonAction(0.87))
//                .afterTime(4.0, autoIntakeServoAxonAction(0.97))
//                .afterTime(5.5, autoIntakeServoAxonAction(0.87))
             //   .afterTime(5.7, autoIntakeServoAxonAction(0.97))
//                .afterTime(5, autoIntakeSliderAction(0,1,0))  //pull/keep Intake slider in before getting to wall
//                .afterTime(10, autoIntakeSliderAction(0,1,0))  //pull/keep Intake slider in before getting to wall

                /** 1st move back from submersible pole--BY SPLINE THEN STRAFE to push Sample 4,5,6 Home*/
//                .splineToLinearHeading(new Pose2d(24,-32, Math.toRadians(60)),         //pushSample6Home, midway by Spline
//                        Math.toRadians(60), velFast, accMedium)
                .splineToLinearHeading(new Pose2d(20,-36, Math.toRadians(60)),         //pushSample6Home, midway by Spline
                        Math.toRadians(60), velFast, accMedium)
                .strafeToLinearHeading(new Vector2d(22, -37), Math.toRadians(60), velFast, accMedium)
                .stopAndAdd(autoIntakeServoAxonAction(0.97))
                .waitSeconds(0.5)
             //   .turnTo(Math.toRadians(-20))
                .strafeToLinearHeading(new Vector2d(20, -50), Math.toRadians(-20), velFast, accMedium)
                .stopAndAdd(autoIntakeServoAxonAction(0.87))
                //      .waitSeconds(0.5)

                .strafeToLinearHeading(new Vector2d(28, -35), Math.toRadians(50), velFast, accMedium)
                .stopAndAdd(autoIntakeServoAxonAction(0.97))
                .waitSeconds(0.35)
//                .turnTo(Math.toRadians(-20))
                .strafeToLinearHeading(new Vector2d(28, -50), Math.toRadians(-20), velFast, accMedium)
                .stopAndAdd(autoIntakeServoAxonAction(0.87))
                //      .waitSeconds(0.5)


                .strafeToLinearHeading(new Vector2d(35, -32), Math.toRadians(45), velFast, accMedium)
                .stopAndAdd(autoIntakeServoAxonAction(0.97))
                .waitSeconds(0.35)
             //   .turnTo(Math.toRadians(-30))
                .strafeToLinearHeading(new Vector2d(35, -50), Math.toRadians(-20), velFast, accMedium)
                //      .waitSeconds(0.5)
//                .setReversed(true)
//                .strafeTo(new Vector2d(62, -8), velFast, accMedium)     //toward Sample6
                .waitSeconds(10)
                /** push sample 6 home and goto Wall */
                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(62,-34, Math.toRadians(-90)),         //pushSample6Home, midway by Spline
//                        Math.toRadians(-90), velFast, accMedium)
//                .splineToLinearHeading(new Pose2d(40,-57, Math.toRadians(-90)),         //pushSample6Home, midway by Spline
//                        Math.toRadians(-90), velFast, accMedium)
//                .splineToLinearHeading(new Pose2d(40,-57.5, Math.toRadians(-90)),         //pushSample6Home, midway by Spline
//                        Math.toRadians(-90), velFast, accMedium)
                .splineToLinearHeading(new Pose2d(40,-64.5, Math.toRadians(-90)),       //pushSample6Home, by Spline to Wall
                        Math.toRadians(-90), velFast, accMedium);
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"





//-----------cycle 1 to score Specimen1
        /** already Spec1; now score Spec1 */
        TrajectoryActionBuilder firstScoreSpec =  combine_preloadMove_BackPush2SpecToHome_CollectSpec1.endTrajectory().fresh() //collectSpec1.endTrajectory().fresh()
                .setReversed(true)                  //need this for both test cases as robot is moving backward
                /********************TO BE TESTED more diagonal path to Score Spec1 */
//                .splineToLinearHeading(new Pose2d(38, -61.5, Math.toRadians(-90)), Math.toRadians(135), velFast, accMedium) //splineToConstantHeading(new Vector2d(38,-61.5),Math.toRadians(135),
//                                                                    //more more diagonal path to Submersible
//                //.splineToConstantHeading(new Vector2d(3,-36),Math.toRadians(90))      //COMMENTING OUT TO TEST more diagonal path to Score Spec1
//                .splineToLinearHeading(new Pose2d(3, -35.5, Math.toRadians(-90)), Math.toRadians(90), velFast, accMedium)  //splineToConstantHeading(new Vector2d(3,-35.5),Math.toRadians(90), velFast, accMedium)      //straighten out robot with another spline to submersible
//                .splineToLinearHeading(new Pose2d(3, -35, Math.toRadians(-90)), Math.toRadians(90), velFast, accMedium); //splineToConstantHeading(new Vector2d(3,-35), Math.toRadians(90), velFast, accMedium);      //straighten out robot with another spline to submersible
                // scored specimen X = 7.5 (preload), now 3, -1, -5, -9)

            //    .splineToConstantHeading(new Vector2d(38,-61.5),Math.toRadians(135), velFast, accMedium)
                .splineToConstantHeading(new Vector2d(3,-36),Math.toRadians(90))  // scored specimen X = 7.5 (preload), 3, now -1, -5, -9)
                .splineToConstantHeading(new Vector2d(3,-35.5),Math.toRadians(90), velFast, accMedium)
                .splineToConstantHeading(new Vector2d(3,-33), Math.toRadians(90), velFast, accMedium);


        /** Collect Spec2 and score Spec2 */
        TrajectoryActionBuilder collectSpec2 = firstScoreSpec.endTrajectory().fresh()
                .stopAndAdd(new AutoClawAction(clawOpen_Pos,clawOpen_pauseTimeSecond))        //open claw
                .afterTime(0.6, autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0))    //rotate outtake arm to ready position to grab specimen on wall
                .afterTime(0.6, autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos))        //extend out arm out to grab specimen on wall
                .afterTime(0.6, autoOuttakeSliderAction(wallPickup_SliderPos, 1))           //=0 outtake slider position to pickup specimen from wall

                /**********************TO BE TESTED*/
             //   .strafeToLinearHeading(new Vector2d(40,-64.5),
             ///           Math.toRadians(-90), velFast, accMedium);                           //1st move back from submersible pole; **test direct strafe, not spline
                /** Comment below out to test */
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(3,-40),
                        Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(38,-62.5),Math.toRadians(-90))      //spline to more to near wall
                .splineToConstantHeading(new Vector2d(38,-63),Math.toRadians(-90))        //spline to more to near wall
                .splineToConstantHeading(new Vector2d(38,-64.5),Math.toRadians(-90));     //spline to more to near wall
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"


//-----------cycle 2 to score Specimen2
        TrajectoryActionBuilder secondScoreSpec = drive.actionBuilder(collectPose) //collectSpec2.endTrajectory().fresh()
                .setReversed(true)                                                       //need this as robot is moving backward
                .splineToConstantHeading(new Vector2d(-1,-36),Math.toRadians(90))  // scored specimen X = 7.5 (preload), 3, now -1, -5, -9)
                .splineToConstantHeading(new Vector2d(-1,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-1,-33),Math.toRadians(90));


        /** Collect Spec3 and score Spec3 */
        TrajectoryActionBuilder collectSpec3 = secondScoreSpec.endTrajectory().fresh()
                .stopAndAdd(new AutoClawAction(clawOpen_Pos,clawOpen_pauseTimeSecond))        //open claw
                .afterTime(0.6, autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0))    //rotate outtake arm to ready position to grab specimen on wall
                .afterTime(0.6, autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos))        //extend out arm out to grab specimen on wall
                .afterTime(0.6, autoOuttakeSliderAction(wallPickup_SliderPos, 1))           //=0 outtake slider position to pickup specimen from wall

                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-1,-40),
                        Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(38,-62.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38,-63),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38,-64.5),Math.toRadians(-90));
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"

//-----------cycle 3 to score Specimen3
        TrajectoryActionBuilder thirdScoreSpec = drive.actionBuilder(collectPose) //collectSpec3.endTrajectory().fresh()
                .setReversed(true)                                                          //need this as robot is moving backward
                .splineToConstantHeading(new Vector2d(-5,-36),Math.toRadians(90))   // scored specimen X = 7.5 (preload), 3, -1, now -5, -9)
                .splineToConstantHeading(new Vector2d(-5,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-5,-33),Math.toRadians(90));


        /** Collect Spec4 and score Spec4 */
        TrajectoryActionBuilder collectSpec4 =  drive.actionBuilder(collectPose)  // thirdScoreSpec.endTrajectory().fresh()
                .stopAndAdd(new AutoClawAction(clawOpen_Pos,clawOpen_pauseTimeSecond))        //open claw
                .afterTime(0.6, autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0))    //rotate outtake arm to ready position to grab specimen on wall
                .afterTime(0.6, autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos))        //extend out arm out to grab specimen on wall
                .afterTime(0.6, autoOuttakeSliderAction(wallPickup_SliderPos, 1))           //=0 outtake slider position to pickup specimen from wall

                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-5,-40),
                        Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
                .splineToConstantHeading(new Vector2d(38,-62.5),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38,-63),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(38,-64.5),Math.toRadians(-90));
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"

//-----------cycle 4 to score Specimen4
        TrajectoryActionBuilder fourthScoreSpec  = collectSpec4.endTrajectory().fresh()
                .setReversed(true)                                                              //need this as robot is moving backward
                .splineToConstantHeading(new Vector2d(-9,-36),Math.toRadians(90))   // scored specimen X = 7.5 (preload), 3, -1, -5, now -9)
                .splineToConstantHeading(new Vector2d(-9,-35.5),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-9,-35),Math.toRadians(90));


/** Park Home */
        /** Stop after 3 specimen score (with preload) and skip 4th specimen score */
        TrajectoryActionBuilder endAtHome = thirdScoreSpec.endTrajectory().fresh()
        //TrajectoryActionBuilder endAtHome = fourthScoreSpec.endTrajectory().fresh()
                .stopAndAdd(new AutoClawAction(clawOpen_Pos,clawOpen_pauseTimeSecond))        //open claw
                .afterTime(1, autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0))    //rotate outtake arm to ready position to grab specimen on wall
                .afterTime(1, autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos))        //extend out arm out to grab specimen on wall
                .afterTime(1, autoOuttakeSliderAction(wallPickup_SliderPos, 1))           //=0 outtake slider position to pickup specimen from wall

                .strafeToLinearHeading(new Vector2d(40,-63.5),
                        Math.toRadians(-90), velFast, accMedium)                           //1st move back from submersible pole
                .setTangent(-90)
                .lineToY(-65);

//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(-5,-40),
//                        Math.toRadians(-90), velFast, accMedium)  //1st move back from submersible pole
//                .splineToConstantHeading(new Vector2d(38,-62.5),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(38,-63),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(38,-64.5),Math.toRadians(-90));
                // starting pose Y = -63, so need to go back more to wall, so -63.5?  TODO: need to reset Y as "wall squaring method"



        //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {
            //         RRAutoCoreInitLoop();
            /**PLACE Huskylens code here for testing purpose*/

            /**then PLACE LED reading here for reading purpose*/

            autoRobot.Outtake.extendIN();
            autoRobot.Outtake.outtakeArmAxon.setPosition(0.28);
            autoRobot.Outtake.closeClaw();
            autoRobot.Intake.intakeINSIDEBOT();
            autoRobot.Intake.intakeSlideIN();

            telemetry.addLine("Initialized");
            telemetry.addData("Alliance Color/Mode: ", AllianceBasketOrSpecimen);
            telemetryA.addData("Starting X = ", beginPose.position.x);
            telemetryA.addData("Starting Y = ", beginPose.position.y);
            telemetryA.addData("Starting Heading (Degrees) = ", Math.toDegrees(beginPose.heading.toDouble()));


        }

        waitForStart();
        if (isStopRequested()) {//return;
            ////TODO:  methods to constantly write into into our AUTOstorage class to transfer to Teleop
            //
            sleep(1000);
        }


        //*********STARTING MAIN PROGRAM****************************************************************
        //opmodeTimer.resetTimer();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                autoOuttakeArmAxonAction(scoring_OuttakeArmAxon_ScoringPos,0),        //rotate outtake arm to scoring position
                                autoOuttakeSliderAction(scoring_OutakeSlider_ScoringPos,1),      //raise outtake  slider to scoring position
                                preloadScore.build()               //move forward to submersible; //**open claw; //1st move back from submersible
                        ),

                        new ParallelAction(
                                //autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0),      //TODO: rotate outtake arm to ready position to grab specimen on wall
                                //autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos),           //TODO: extend out arm out (need to this to the place after preloadscore
                                //autoOuttakeSliderAction(wallPickup_SliderPos, 1),       //TODO: ?outtake slider position to pickup specimen from wall
                                //preloadMoveBack.build()
                                /**comment out before but push sample homes were use as one more, then
                                 comment out 303 and 311-314, and uncomment out 306 */
                                combine_preloadMove_BackPush2SpecToHome_CollectSpec1.build()
                        ),


                        /**PUSH SAMPLES HOME */
                        //gotoSample4.build(),
                        //pushSample4HomeThenToSample5.build(),
                        //pushSample5HomeThenToSample6.build(),         //skipping push of specimen 6 home.
                        //pushSample6Home.build(),

                        /**COLLECT FIRST SPEC AND SCORE */
                        //collectSpec1.build(),
                        autoClawAction(clawClose_Pos, clawClose_pauseTimeSecond),                     //close claw
                        new ParallelAction(
                                autoOuttakeSliderAction(scoring_OutakeSlider_ScoringPos, 1),        //move slider up to scoring position
                                autoouttakeExtensionAction(scoring_OuttakeExtension_ScoringPos),          //bring arm in to scoring position
                                autoOuttakeArmAxonAction(scoring_OuttakeArmAxon_ScoringPos,0),    //rotate to specimen score position
                                firstScoreSpec.build()   //goto submersible pole; open claw; 1st move back from pole
                        ),

                        /**GO COLLECT 2ND SPEC AND SCORE */
                        new ParallelAction(
                                //autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0),      //rotate outtake arm to ready position to grab specimen on wall
                                //autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos),                 //extend out arm out (need to this to the place after preloadscore
                                //autoOuttakeSliderAction(wallPickup_SliderPos, 1),               //=0 outtake slider position to pickup specimen from wall
                                collectSpec2.build()  //open claw; move back from submersible; delay rotate arm in, extend to wall, lower slider; THEN goto wall
                        ),
                        autoClawAction(1, clawClose_pauseTimeSecond),                     //close claw
                        new ParallelAction(
                                autoOuttakeSliderAction(scoring_OutakeSlider_ScoringPos, 1),        //move slider up
                                autoouttakeExtensionAction(scoring_OuttakeExtension_ScoringPos),          //bring arm into robot to scoring position
                                autoOuttakeArmAxonAction(scoring_OuttakeArmAxon_ScoringPos,0),  //rotate to specimen score position
                                secondScoreSpec.build()                               //goto submersible pole; open claw; //1st move back from pole
                        ),


                        /**COLLECT 3rd SPEC AND SCORE */
                        new ParallelAction(
                                //autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0),      //rotate outtake arm to ready position to grab specimen on wall
                                //autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos),                 //extend out arm out (need to this to the place after preloadscore
                                //autoOuttakeSliderAction(wallPickup_SliderPos, 1),               //=0 outtake slider position to pickup specimen from wall
                                collectSpec3.build()                        //goto wall
                        ),
                        autoClawAction(clawClose_Pos, clawClose_pauseTimeSecond),       //close claw
                        new ParallelAction(
                                autoOuttakeSliderAction(scoring_OutakeSlider_ScoringPos, 1),        //=1240? move slider up
                                autoouttakeExtensionAction(scoring_OuttakeExtension_ScoringPos),          // = 0.85?  bring arm into robot
                                autoOuttakeArmAxonAction(scoring_OuttakeArmAxon_ScoringPos,0),  // = 0.90?  rotate to specimen score position
                                thirdScoreSpec.build()      //goto submersible pole; open claw; //1st move back from pole
                        ),


/** SKIPPING 4th SPECIMEN SCORE--preload specimen is addition specimen score  */
//                        /**COLLECT 4rd SPEC AND SCORE */
//                        new ParallelAction(
//                                //autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0),      //rotate outtake arm to ready position to grab specimen on wall
//                                //autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos),                 //extend out arm out (need to this to the place after preloadscore
//                                //autoOuttakeSliderAction(wallPickup_SliderPos, 1),               //=0 outtake slider position to pickup specimen from wall
//                                collectSpec4.build()                        //goto wall
//                        ),
//                        autoClawAction(clawClose_Pos, clawClose_pauseTimeSecond),       //close claw
//                        new ParallelAction(
//                                autoOuttakeSliderAction(scoring_OutakeSlider_ScoringPos, 1),        //=1240? move slider up
//                                autoouttakeExtensionAction(scoring_OuttakeExtension_ScoringPos),          // = 0.85?  bring arm into robot
//                                autoOuttakeArmAxonAction(scoring_OuttakeArmAxon_ScoringPos,0),  // = 0.90?  rotate to specimen score position
//                                fourthScoreSpec.build()      //goto submersible pole; open claw; //1st move back from pole
//                        ),



                        /**Park Home*/
                        new ParallelAction(
                                autoOuttakeArmAxonAction(wallPickup_OuttakeArmAxonPos, 0),      //TODO: =0.28?  rotate outtake arm to ready position to grab specimen
                                autoouttakeExtensionAction(wallPickup_OuttakeExtensionPos),                 //TODO: = 0.85? extend out arm out (need to this to the place after preloadscore
                                autoOuttakeSliderAction(0, 1),               //TODO: = 0? ?outtake slider position to pickup specimen from wall
                                endAtHome.build() //goto home
                        )



                )
        );



        drive.updatePoseEstimate();
        Pose2d poseEnd = drive.localizer.getPose();
        telemetryA.addData("x", poseEnd.position.x);
        telemetryA.addData("y", poseEnd.position.y);
        telemetryA.addData("heading (deg)", Math.toDegrees(poseEnd.heading.toDouble()));
//        RRAutoCoreTelemetryDuringteleOp();          //show robot drawing on FTC Dashboard //TODO: add in the AUTOcore method to transfer data to teleOp
        telemetryA.update();



/**TRANSFER SUBSYSTEM positions at end of AUTO TO TELEOP */
        AUTOstorageConstant.AllianceBasketOrSpecimen = AllianceBasketOrSpecimen;
        //Intake servo and motor position position:
        AUTOstorageConstant.autoEnd_Intake_intakeSlides_MotorPosition = autoRobot.Intake.intakeSlides.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Intake_intakeServoAxon_ServoPosition = autoRobot.Intake.intakeServoAxon.getPosition();  //TODO: confirm name
        //Outtake servo and motor position:
        AUTOstorageConstant.autoEnd_Outtake_outtakeLeftSlide_MotorPosition = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeRightSlide_MotorPosition = autoRobot.Outtake.outtakeRightSlide.getCurrentPosition();
        AUTOstorageConstant.autoEnd_Outtake_outtakeArmAxon_ServoPosition = autoRobot.Outtake.outtakeArmAxon.getPosition();  //TODO: confirm name
        AUTOstorageConstant.autoEnd_Outtake_claw_ServoPosition = autoRobot.Outtake.claw.getPosition();
        //Drivetrain: Pose and heading
        AUTOstorageConstant.autoEndheadingIMU_yawDEG = autoRobot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        AUTOstorageConstant.autoPoseEnd =  poseEnd;
        AUTOstorageConstant.autoEndX = poseEnd.position.x;
        AUTOstorageConstant.autoEndY = poseEnd.position.y;
        AUTOstorageConstant.autoEndHeadingDEG = Math.toDegrees(poseEnd.heading.toDouble());




        sleep(100000);
    }






    private Action SleepAction(long milliseconds) {
        sleep(milliseconds);
        return null;
    }


    /**ACTION METHODS
     * to implementing actions with SequentialAction
     *         Actions.runBlocking(
     *                 new SequentialAction(
     *                         trajectoryActionChosen,
     *                         AutoOuttakeSliderHighBasket(),
     *                         trajectory....
     *                         .waitSeconds(3)
     *                         AutoOuttakeSliderHighBasket(),
     *                         trajectoryActionCloseOut
     *                 )
     *         );
     *to implement by
     *
     */

    /**
     * positions of OuttakeSlider: 0=ground, max/high basket=3400; high specimen bar=1300
     */
    public class AutoOuttakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;

        public AutoOuttakeSliderAction(int position, double power) {
            this.desirePosition = position;
            this.desiredPower = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.leftSlideSetPositionPower(desirePosition, desiredPower);
                autoRobot.Outtake.rightSlideSetPositionPower(desirePosition, desiredPower);
                initialized = true;
            }
            double positionOuttakeLeftSlide = autoRobot.Outtake.outtakeLeftSlide.getCurrentPosition();
            packet.put("Outtake slider-left position", positionOuttakeLeftSlide);
            if (Math.abs((positionOuttakeLeftSlide - desirePosition)) > 20) {
                return true;
            } else {
                return false;
            }
        }
    }

    public Action autoOuttakeSliderAction(int position, double power) {
        return new AutoOuttakeSliderAction(position, power);
    }


    public class AutoOuttakeSliderHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoRobot.Outtake.slidersOnlyHighBasket();
            return false;
        }
    }

    public Action autoOuttakeSliderHighBasketAction() {
        return new AutoOuttakeSliderHighBasketAction();
    }


    /**
     * safe range for OuttakeArmAxon = 0.29 to 0.35 to 0.37 to 0.68 to 0.86
     */
    public class AutoOuttakeArmAxonAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;
        double pauseTimeSec;

        public AutoOuttakeArmAxonAction(double position, double pauseTimeSec) {
            this.desirePosition = position;
            this.pauseTimeSec = pauseTimeSec;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.outtakeArmAxon.setPosition(desirePosition);
                initialized = true;
            }
            return timer.seconds() < pauseTimeSec;
        }
    }

    public Action autoOuttakeArmAxonAction(double position, double pauseTimeSec) {
        return new AutoOuttakeArmAxonAction(position, pauseTimeSec);
    }


    /**
     * safe range for  outtakeExtension =
     */
    public class AutoouttakeExtensionAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public AutoouttakeExtensionAction(double position) {
            this.desirePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.outtakeExtension.setPosition(desirePosition);
                initialized = true;
            }
            return false;
        }
    }

    public Action autoouttakeExtensionAction(double position) {
        return new AutoouttakeExtensionAction(position);
    }


    /**
     * safe range for cLAW =
     */
    public class AutoClawAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;
        double pauseTime;
        public AutoClawAction(double position, double pauseTime) {
            this.desirePosition = position;
            this.pauseTime = pauseTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Outtake.claw.setPosition(desirePosition);
                initialized = true;
            }
            return timer.seconds() < pauseTime;
        }
    }

    public Action autoClawAction(double position, double pauseTime) {
        return new AutoClawAction(position, pauseTime);
    }


    /**
     * safe range for IntakeSlider MOTOR
     */
    public class AutoIntakeSliderAction implements Action {
        private boolean initialized = false;
        int desirePosition;
        double desiredPower;
        ElapsedTime timer;
        double pauseTime;
        public AutoIntakeSliderAction(int position, double power, double pauseTime) {
            this.desirePosition = position;
            this.desiredPower = power;
            this.pauseTime = pauseTime;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeSlideSetPositionPower(desirePosition, desiredPower);
                initialized = true;
            }
            double positionIntakeSlide = autoRobot.Intake.intakeSlides.getCurrentPosition();
            packet.put("Intake slider position", positionIntakeSlide);
            if (Math.abs((positionIntakeSlide - desirePosition)) > 20) {
                return true;
            } else {
                return timer.seconds() < pauseTime;
            }

        }
    }

    public Action autoIntakeSliderAction(int position, double power, double pauseTime) {
        return new AutoIntakeSliderAction(position, power, pauseTime);
    }


    /**
     * safe range for intakeServoAxon =
     */
    public class AutoIntakeServoAxonAction implements Action {
        private boolean initialized = false;
        double desirePosition;
        ElapsedTime timer;

        public AutoIntakeServoAxonAction(double position) {
            this.desirePosition = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeServoAxon.setPosition(desirePosition);
                initialized = true;
            }
            return false;
        }
    }
    public Action autoIntakeServoAxonAction(double position) {

        telemetry.addLine("autoIntakeServoAxonAction");
        return new AutoIntakeServoAxonAction(position);
    }


    public class AutoIntakeSpinerAction implements Action {
        private boolean initialized = false;
        double power;
        ElapsedTime timer;
        double pauseTimeSec;
        public   AutoIntakeSpinerAction(double power, double pauseTimeSec) {

            this.power = power;
            this.pauseTimeSec = pauseTimeSec;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                autoRobot.Intake.intakeLeftWheel.setPower(power);
                autoRobot.Intake.intakeRightWheel.setPower(-power);
                initialized = true;
            }
            return timer.seconds() < pauseTimeSec;
        }
    }
    //power:  positive = intake split OUT sample
    public Action autoIntakeSpiner(double power, double pauseTimeSec){
        return new AutoIntakeSpinerAction(power, pauseTimeSec);
    }

    public class AutoDiplayAction implements Action {
        private boolean initialized = false;
        String teleData;
        ElapsedTime timer;

        public AutoDiplayAction(String teleData) {
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer = new ElapsedTime();
                telemetry.addLine(teleData);
                telemetry.update();
                initialized = true;
            }
            return false;
        }
    }
    //power:  positive = intake split OUT sample
    public Action autoDiplayAction(String teleData){
        return new AutoDiplayAction(teleData);
    }


}




////Debugging messages
//void autoDebug(int myLevel, String myName, String myMessage) {
//    if (debugLevel > myLevel)  {
//        telemetryA.addData("**DEBUG**: " + myName, myMessage);
//        telemetryA.update();
//    }
//    if ((1000 > myLevel) || (debugLevel > myLevel)) {
//        RobotLog.i("**DEBUG** == " + myName + ": " + myMessage);
//    }
//}
//
////Log messages that are always shown
//void autoLog(String myName, String myMessage){
//    telemetryA.addData(myName, myMessage);
//    telemetryA.update();
//    RobotLog.i("LOG == " + myName + ": " + myMessage);
//}