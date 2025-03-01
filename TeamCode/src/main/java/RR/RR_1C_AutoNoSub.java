package RR;

/** Dominic:  updated 2/20/2025
 *  ED: updated 2/21/2025 morning
 *  2/22/2025Sat.  Dominic: Alliance without AUTO.
 *                          Auto BLUE and RED with Submersible LimeLight and light sensor to spit out wrong sample pickup
 *                          are in separate class
 *                          TODO: NEED TO TUNE and ADJUST BASKET SLIDER POSITION due to slider restrung
 * */


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
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import Hardware.HardwareNoDriveTrainRobot;

@Config
//@Autonomous(name = "RR_1C_Auto_ NoSub v1", group = "Auto")


    public class RR_1C_AutoNoSub extends LinearOpMode {

        //TODO: setup initial position for all subsystems
        //    public static double autoEnd_SliderMotorPosition,
        //            autoEnd_Slider_ServoArmPosition;


        HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();    //TODO: will this interfere with follower(hardwareMap)? in .init


        //-------------------------------------------------------------------------------------------------
        @Override
        public void runOpMode() throws InterruptedException {
            double sliderPower = 1;

            Limelight3A limelight;
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            telemetry.setMsTransmissionInterval(11); // This sets how often we ask Limelight for data (100 times per second)
            limelight.pipelineSwitch(1);
//        limelight.start(); // This tells Limelight to start looking!

            AutoOuttakeSliderAction autoOuttakeSliderAction = null;
            autoRobot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class

            String AllianceBasketOrSpecimen = "1C_AUTO_REDBasket_NoSUB";
            Pose2d beginPose = new Pose2d(-32, -62, Math.toRadians(0));     //TODO: would overide this for each case

            int debugLevel = 499;
            Telemetry telemetryA;
            Timer pathTimer, actionTimer, opmodeTimer;
            Pose2d pose;

            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();
            telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            //autoDebug(500, "Auto:Init", "DONE");
            telemetryA.update();

            //adjust these settings as needed for use in the trajectory codes
            VelConstraint velSlow = new TranslationalVelConstraint(15);
            VelConstraint velFast = new TranslationalVelConstraint(45);
            AccelConstraint accSlow = new ProfileAccelConstraint(-15, 15);
            AccelConstraint accFast = new ProfileAccelConstraint(-45, 45);

            double readyPosition = 0.43;
            double grabPosition = 0.33;
            double intakeAxonPosition = 0.65;
            double sweeperIn = 0.09;
            double sweeperOUT = 0.5;

            Vector2d grabPosePosition = new Vector2d(-61, -48);
            Vector2d BasketDropPosition = new Vector2d(-55.5, -52.5);

            /**  .lineToX(24.5,velSlow,accSlow)        //example on how to use these in drive.actionBuilder */

            TrajectoryActionBuilder preScore = drive.actionBuilder(beginPose)
                    .setTangent(45)
                    .waitSeconds(0)
                    .strafeToSplineHeading(new Vector2d(-57.5, -52.5), Math.toRadians(45));

            TrajectoryActionBuilder grabPose = preScore.endTrajectory().fresh()
                    .turnTo(Math.toRadians(80));
//                .strafeToSplineHeading(grabPosePosition, Math.toRadians(71));

            TrajectoryActionBuilder turnToBasket3 = grabPose.endTrajectory().fresh()
                    .waitSeconds(0.6)
                    .turnTo(Math.toRadians(45));

            TrajectoryActionBuilder grabPose2 = turnToBasket3.endTrajectory().fresh()
                    .turnTo(Math.toRadians(106));
//                .strafeToSplineHeading(grabPosePosition, Math.toRadians(97));

            TrajectoryActionBuilder turnToBasket20 = grabPose2.endTrajectory().fresh()
                    .waitSeconds(0.5)
                    .turnTo(Math.toRadians(45));

            TrajectoryActionBuilder grabPose3 = turnToBasket3.endTrajectory().fresh()
                    .turnTo(Math.toRadians(129));
//                .strafeToSplineHeading(grabPosePosition, Math.toRadians(124));

            TrajectoryActionBuilder turnToBasket10 = grabPose3.endTrajectory().fresh()
                    .waitSeconds(0.5)
                    .turnTo(Math.toRadians(45));

            TrajectoryActionBuilder turnToPreload = turnToBasket10.endTrajectory().fresh()
                    .strafeToSplineHeading(new Vector2d(-47, -50), Math.toRadians(350));

            TrajectoryActionBuilder turnToBasket = turnToBasket10.endTrajectory().fresh()
                    .strafeToSplineHeading(new Vector2d(-57, -54), Math.toRadians(45));

            TrajectoryActionBuilder submersible = turnToBasket3.endTrajectory().fresh()
                    .setTangent(45)
                    .splineToSplineHeading(new Pose2d(-22.5, 0, 0), Math.toRadians(35));




            //*****LOOP wait for start, INIT LOOP***********************************************************
            while (!isStarted() && !isStopRequested()) {
                //         RRAutoCoreInitLoop();
                autoRobot.Outtake.extendIN();
                autoRobot.Outtake.outtakeArmAxon.setPosition(0.28);
                autoRobot.Outtake.closeClaw();
                autoRobot.Intake.intakeINSIDEBOT();

                telemetryA.addLine("Initialized");
                telemetryA.addData("Alliance Color/Mode: ", AllianceBasketOrSpecimen);
                telemetryA.addData("Starting X = ", beginPose.position.x);
                telemetryA.addData("Starting Y = ", beginPose.position.y);
                telemetryA.addData("Starting Heading (Degrees) = ", Math.toDegrees(beginPose.heading.toDouble()));

//            LLResult result = limelight.getLatestResult();
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            for (LLResultTypes.ColorResult cr : colorResults) {
//                if (cr.getTargetXDegrees() > 10) {
//                    telemetryA.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//                }
//            }
//            telemetryA.update();

            }

            waitForStart();
            if (isStopRequested()) {//return;
                ////TODO:  methods to constantly write into into our AUTOstorage class to transfer to Teleop
                //
                sleep(1000);
            }

            //*********STARTING MAIN PROGRAM****************************************************************
            //      opmodeTimer.resetTimer();
            //{
            Actions.runBlocking(
                    new SequentialAction(
                            //Raise Outtake slider and Move Robot to basket
                            autoOuttakeSliderHighBasketAction(),
                            autoOuttakeArmAxonAction(0.77, 0),
                            autoouttakeExtensionAction(0.85, 0),
                            preScore.build(),

                            /** Sample 1*/
                            autoClawAction(0, 0.15),
                            autoouttakeExtensionAction(1, 0),
                            autoOuttakeArmAxonAction(readyPosition, 0),
//
                            autoIntakeServoAxonAction(0.97),
                            autoIntakeSpiner(-1, 0.1),
                            new ParallelAction(
                                    autoOuttakeSliderAction(0, 1),
                                    //move to first sample
                                    grabPose.build()
                            ),
                            autoIntakeSliderAction(290, sliderPower, 0),

                            /*NEXT STEP*/
                            autoIntakeSliderAction(1, sliderPower, 0),
                            autoIntakeServoAxonAction(intakeAxonPosition),
                            autoIntakeSpiner(-1, 0.5),
                            autoOuttakeArmAxonAction(grabPosition, 0.1),
                            autoClawAction(0.3, 0.2),
                            new ParallelAction(
                                    autoOuttakeSliderHighBasketAction(),
                                    turnToBasket3.build(),
                                    autoOuttakeArmAxonAction(0.77, 0),
                                    autoouttakeExtensionAction(0.8, 0.85)
                            ),
//
                            /** Sample 2*/
                            autoClawAction(0, 0.1),
                            autoouttakeExtensionAction(1, 0),
                            autoOuttakeArmAxonAction(readyPosition, 0),

                            autoIntakeServoAxonAction(0.97),
                            autoIntakeSpiner(-1, 0.05),
                            //move to first sample
                            new ParallelAction(
                                    autoOuttakeSliderAction(0, 1),
                                    //move to first sample
                                    grabPose2.build()
                            ),
                            autoIntakeSliderAction(300, sliderPower, 0),

                            /*NEXT STEP*/
                            autoIntakeSliderAction(1, sliderPower, 0),
                            autoIntakeServoAxonAction(intakeAxonPosition),
                            autoIntakeSpiner(-1, 0.5),
                            autoOuttakeArmAxonAction(grabPosition, 0.1),
                            autoClawAction(0.3, 0.2),
                            new ParallelAction(
                                    autoOuttakeSliderHighBasketAction(),
                                    turnToBasket20.build(),
                                    autoOuttakeArmAxonAction(0.77, 0),
                                    autoouttakeExtensionAction(0.8, 0.6)
                            ),

                            /**Sample 3*/
                            autoClawAction(0, 0.1),
                            autoouttakeExtensionAction(1, 0),
                            autoOuttakeArmAxonAction(readyPosition, 0),

                            autoIntakeServoAxonAction(0.97),
                            autoIntakeSpiner(-1, 0.05),
                            //move to first sample
                            new ParallelAction(
                                    autoOuttakeSliderAction(0, 1),
                                    //move to first sample
                                    grabPose3.build()
                            ),
                            autoIntakeSliderAction(305, sliderPower, 0),

                            /*NEXT STEP*/
                            autoIntakeSliderAction(1, sliderPower, 0),
                            autoIntakeServoAxonAction(intakeAxonPosition),
                            autoIntakeSpiner(-1, 0.5),
                            autoOuttakeArmAxonAction(grabPosition, 0.1),
                            autoClawAction(0.3, 0.25),
                            new ParallelAction(
                                    autoOuttakeSliderHighBasketAction(),
                                    turnToBasket10.build(),
                                    autoOuttakeArmAxonAction(0.77, 0),
                                    autoouttakeExtensionAction(0.8, 0.6)
                            ),

                            autoClawAction(0, 0.15),

                            //End of V1
                            autoOuttakeArmAxonAction(readyPosition, 0),
                            autoouttakeExtensionAction(1, 0),
                            autoOuttakeSliderAction(1, 1),

                            //Start of V2
                            autoIntakeServoAxonAction(0.80),
                            turnToPreload.build(),

                            autoIntakeServoAxonAction(0.97),
                            autoIntakeSliderAction(310, sliderPower, 0),
                            autoIntakeSliderAction(0, sliderPower, 0),

                            autoIntakeServoAxonAction(intakeAxonPosition),
                            autoIntakeSpiner(-1, 0.5),
                            autoOuttakeArmAxonAction(grabPosition, 0.1),
                            autoClawAction(0.3, 0.25),
                            new ParallelAction(
                                    autoOuttakeSliderHighBasketAction(),
                                    turnToBasket.build(),
                                    autoOuttakeArmAxonAction(0.77, 0)

                            ),
                            autoouttakeExtensionAction(0.8, 1.25),
                            autoClawAction(0, 0.15),

                            autoOuttakeArmAxonAction(readyPosition, 0),
                            autoouttakeExtensionAction(1, 0),
                            autoOuttakeSliderAction(1, 1),

                            submersible.build(),
                            autoClawAction(0, 0.15),
                            autoouttakeExtensionAction(0.8, 0.),
                            autoOuttakeArmAxonAction(0.37, 0)

                    )
            );

//}

            drive.updatePoseEstimate();
            Pose2d poseEnd = drive.localizer.getPose();
            telemetryA.addData("x", poseEnd.position.x);
            telemetryA.addData("y", poseEnd.position.y);
            telemetryA.addData("heading (deg)", Math.toDegrees(poseEnd.heading.toDouble()));
//        RRAutoCoreTelemetryDuringteleOp();          //show robot drawing on FTC Dashboard //TODO: add in the AUTOcore method to transfer data to teleOp
            telemetryA.update();

            limelight.stop();

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
            double pauseTime;

            public AutoouttakeExtensionAction(double position, double pauseTime) {
                this.desirePosition = position;
                this.pauseTime = pauseTime;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer = new ElapsedTime();
                    autoRobot.Outtake.outtakeExtension.setPosition(desirePosition);
                    initialized = true;
                }
                return timer.seconds() < pauseTime;
            }
        }

        public Action autoouttakeExtensionAction(double position, double pauseTime) {
            return new AutoouttakeExtensionAction(position, pauseTime);
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


        public class AutoIntakeSpiner implements Action {
            private boolean initialized = false;
            double power;
            ElapsedTime timer;
            double pauseTimeSec;
            public   AutoIntakeSpiner(double power, double pauseTimeSec) {

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
            return new AutoIntakeSpiner (power, pauseTimeSec);
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

        public class AutoIntakeSweeperAction implements Action {
            private boolean initialized = false;
            double desirePosition;
            ElapsedTime timer;
            double pauseTimeSec;

            public AutoIntakeSweeperAction(double position, double pauseTimeSec) {
                this.desirePosition = position;
                this.pauseTimeSec = pauseTimeSec;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer = new ElapsedTime();
                    autoRobot.Intake.sweeper.setPosition(desirePosition);
                    initialized = true;
                }
                return timer.seconds() < pauseTimeSec;
            }
        }

        public Action AutoIntakeSweeperAction(double position, double pauseTimeSec) {
            return new AutoIntakeSweeperAction(position, pauseTimeSec);
        }

        //power:  positive = intake split OUT sample
        public Action autoDiplayAction(String teleData){
            return new AutoDiplayAction(teleData);
        }

    }