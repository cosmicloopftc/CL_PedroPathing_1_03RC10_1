import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

import Hardware.HardwareDrivetrain;
import Hardware.HardwareNoDriveTrainRobot;
import RR.AUTOstorageConstant;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import Diagnostic.Datalogger;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.Anyi Lin, Aaron Yang, Harrison Womack - 10158 Scott's Bots, @version 1.0, 3/21/2024

 8/26/2024:  update motor map in this class, in PedroPathing/follower/Follower, in PedroPathing/localization/localizers/ThreeWheelLocalizer
 9/2/2024WT: add FTC dashboard lines similar to those in LocalizerTest
 10/13/2024OT: transfer over PedroPath TeleOp from previous Pedropath tuning
 10/13/2024WT: transfer PedroPath tuning info
 10/14/2024MT/WT: add manual driving, not using PedroPath Follower
 10/24/2024WT: correct HardwareLED class, rename in AdafruitLED object name--12/16/2024---LED HardwareLED() with both regualr LED and Adafruit long LED string
 11/1/2024: Allow for high and low basket scoring with no intaking
 11/23/2024: Allow for more than 1 time intaking + wall intake in start state w/specimen positions
 11/26/2024: Fix the outtake finite state machine conditions
 11/27/2024: Fixed and improved specimen scoring
 11/28/2024: Allow to go from specimen scoring directly back to wall intake
 11/30/2024, 12/17/2024:  Add TelemetryA and Drawing for FTC Dashboard access
 12/17/2024:    add slide motor current draw
 12/18/2024: note HardwareNoDriveTrainRobot = pull all hardware mapping/function into here except for drivetrain motors
 HardwareRobot = pull all hardware mapping/function including drivetrain motors
 Add PedroPathing teleop control (Note: Follower object has the drivetrain motor mapping already),
 Fix option for non-PedroPathing teleop control if desired later
 by using HardwareDrivetrain and HardwareNoDrivetrainRobot separately
 Activate PedroPathing teleop by: 1) use follower.startTeleopDrive() in start() loop and comment out drive comments in loop()
 Use regular driving with the opposite of above.
 12/27/2024: add Color sensor for color, hue, distance based on FIRST external example
 1/1/2025:   Update Diagnostic
 1/3/2025:   Update new PedroPath 1.0.3 teleOp movement
 2/19/2025:  add AUTOconstant of data transfer from end of AUTO to staring pose
 2/20/2025:  add Datalogger Class to Diagnostic folder and incorporate ConceptDataLogger BLUE_TeleOpV1 Class
 For instructions, see the tutorial at the FTC Wiki:  https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging
 Credit to @Windwoes (https://github.com/Windwoes).
 To access the file: connect to Drive Hub and go to : /sdcard/FIRST/java/src/Datalogs, look for the file
 Right-click on a log file, and choose "Download". During the download dialog,
 navigate to your target folder on the laptop, and change the file extension from .txt to .csv.
 This change allows the file to be automatically recognized and imported by spreadsheet programs.
 To graph data: In the spreadsheet, select columns to graphs: Elapsed Time and other data columns.
 select Insert Chart, basic 2-D line style graph option
 TODO: writeDatalog()--need to check if this will slow down run loop--line 341 and line 814
 */



@Config    //need this to allow appearance in FtcDashboard Configuration to make adjust of variables
@TeleOp(group="Primary", name= "BLUE_TeleOpV1.3")

public class BLUE_TeleOpV1 extends OpMode {
    boolean isGP2rightYupdown = false;
    boolean isGP2leftYupdown = false;

    /** For datalog 2/20/2025 */
    VoltageSensor battery;
    String datalogOpModeStatus;
    double batteryStatus;
    int readCount = 0;
    Datalogger datalog;
    String datalogFilename = "myDatalog_001";       // modify name for each run
    ElapsedTime dataTimer;                          // timer object
    int logInterval = 50;                           // target interval in milliseconds

    //runtime to keep track of time each mode is already defined as usual below.
    String allianceColor;               //Value is assigned in init()
    String nonAllianceColor;            //Value is assigned in init()

    public Telemetry telemetryA;
    //boolean endGameRumble45secondsWarningOnce = true;
    //boolean endGameRumble31secondSTARTonce = true;
    boolean endGameRumble16secondsLeftOnce = true;
    boolean endGameRumble8secondsLeftOnce = true;

    Gamepad.RumbleEffect customRumbleEffect;

    //PedroPathing driving:   based on Robot-Centric Teleop  from @author Baron Henderson - 20077 The Indubitables
    // * @version 2.0, 11/28/2024
    private Follower follower;
    //private final Pose startPose = new Pose(0,0,0);  //TODO: Later, reset this to transfer location from Auto
    //converting RR Coordinate to PedroPathing Coordinate
    private final Pose startPose = new Pose(
            70.5 - AUTOstorageConstant.autoEndY,
            AUTOstorageConstant.autoEndX + 70.5,
            AUTOstorageConstant.autoEndHeadingDEG + 90);  //TODO: Later, reset this to transfer location from Auto



    private String sampleColor = "NONE";
    private boolean intakeExtend = false;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    public static double intakeSlidesCurrent;

    //label robot for all hardware except drivetrain; and drivetrain is a separate object
    HardwareNoDriveTrainRobot robot = new HardwareNoDriveTrainRobot();
    HardwareDrivetrain drivetrain = new HardwareDrivetrain();



    enum State{
        START,
        INTAKE,
        TRANSFER,
        OUTTAKE_READY,
        OUTTAKE,
        READY_DOWN
    }
    State state = State.START;



    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime sweepTime = new ElapsedTime();
    private double loopTimeTotal, loopTimeCount;
    private ElapsedTime loopTime = new ElapsedTime();
    private double loopTimeAverMilliSec;

    boolean sweptIn = true;
    double botHeadingImu;
    String drivingOrientation = "robotOriented";   //TODO: as default for Eduardo, but will also reset in init as well.
    double lastTime;
    double imuAngle;
    boolean clawStatusOpen = false;
    String outtakeOption = "";


    //Declare variables for standard driving--not using PedroPath follower method (using Learn JAVA for FTC book)
    double y, x, rx, powerShift;
    //double newForward = 0, newRight = 0, driveTheta = 0, r = 0, powerShift = 0;


    //Declare variables for Color sensor for color, hue, distance
    float colorGain = 15;
    // Once per loop, we will update this hsvValues array.
    // first element (0)= hue, second element (1)=saturation, third element (2)= value.
    // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    final float[] hsvValues = new float[3];
    // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
    // state of the X button on the gamepad
    boolean xButtonPreviouslyPressed = false;
    boolean xButtonCurrentlyPressed = false;


    Gamepad.LedEffect flashingWhite6Sec = new Gamepad.LedEffect.Builder()
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 1, 1, 500) // Show white for 500ms
            .addStep(0, 0, 0, 500)
            .build();

    Gamepad.LedEffect flashingBlue6Sec = new Gamepad.LedEffect.Builder()
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
            .addStep(0, 0, 1, 500) // Show blue for 500ms
            .addStep(0, 0, 0, 500) //
//                .addStep(1, 0, 0, 250) // Show red for 250ms
//                .addStep(0, 1, 0, 250) // Show green for 250ms
//                .addStep(0, 0, 1, 250) // Show blue for 250ms
//                .addStep(1, 1, 1, 250) // Show white for 250ms
            .build();
    Gamepad.LedEffect flashingRed6Sec = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .addStep(1, 0, 0, 500) // Show red for 500ms
            .addStep(0, 0, 0, 500)
            .build();

    private ElapsedTime pathTimer;
    private int pathState;
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.reset();
    }
    double motorPowerDefault;
    boolean isGamepad2_yPressed = false;
    int intakeSliderPresentPosition;
    double intakeSliderCurrentDraw;
    double intakeSliderAngularVelocity;   //TODO: what is the unit
    int outtakeSlider_Right_PresentPosition;
    double outtakeSlider_Right_CurrentDraw;
    double outtakeSlider_Right_AngularVelocity;   //TODO: what is the unit
    String intakeOptionInitLoop = "Intake Slider at zero";
    String outtakeOptionInitLoop = "Outtake Slider at zero";


    //__________________________________________________________________________________________________
    @Override
    public void init() {
        String allianceColor = "BLUE";
        String nonAllianceColor = "RED";
        motorPowerDefault = 0.8;


        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        robot.init(hardwareMap);   //for all hardware except drivetrain.  note hardwareMap is default and part of FTC Robot Controller HardwareMap class
        robot.imu.resetYaw();      //reset the IMU/Gyro angle with each match.
        drivetrain.init(hardwareMap);   //for drivetrain only

        /** For datalog 2/20/2025 */
        datalogOpModeStatus = "INIT";
        battery = hardwareMap.voltageSensor.get("Control Hub");
        datalog = new Datalogger(datalogFilename);      // Initialize the datalog
        dataTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);// Instantiate datalog timer.
        datalog.addField("Count");
        datalog.addField("OpModeStatus");
        datalog.addField("Elapsed Time (mS)");
        datalog.addField("battery Voltage");
        datalog.addField("IMU Heading Angle (Deg)");
        datalog.addField("loop Time (mS)");
        datalog.firstLine();                        // end first line (row)


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        follower.startTeleopDrive();
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();

        telemetryA.addData("Gain", colorGain);
        telemetryA.addLine("");
        // Tell sensor desired gain value (normally you would do this during initialization, not during loop)
        robot.Sensor.colorIntake.setGain(colorGain);

        telemetryA.addData(">", "Hardware Initialization complete");
        telemetryA.update();
        runtime.reset();

    }


    @Override
    public void init_loop() {
        loopTime.reset();
        bulkReadTELEOP();
        botHeadingImu = imuAngle;








/**testing zeroing of Outtake slider to ground position*****/
        telemetryA.addLine("testing");
        telemetryA.addLine("gamepad2 y/a = slider up/dowm");
        telemetryA.addLine("...dpad up/down = slider out/in");


        //TODO: if right outtake slider is <10 position and
        //      angular velocity is <5 degree/sec <0.02 (1 degree = 0.175 radian), then reset slider to ZERO position, need to test and adjust
        if ((outtakeSlider_Right_PresentPosition < 20) && (Math.abs(outtakeSlider_Right_AngularVelocity)<1) && !(gamepad2.right_stick_y > 0.1) && !(gamepad2.left_stick_y > 0.1)){
            robot.Outtake.outtakeLeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.Outtake.outtakeRightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //robot.Outtake.outtakeLeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  //TODO: do we need this line?
            //robot.Outtake.outtakeRightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //TODO: do we need this line?
            outtakeOptionInitLoop = "reset Outtake Slider to zero";
        }

        if(gamepad2.y) {
            robot.Outtake.slidersOnlyHighBasket();
            outtakeOptionInitLoop = "Outtake Slider--highBasket";
        }
        if (gamepad2.a) {
            robot.Outtake.groundPosition();
            outtakeOptionInitLoop = "Outtake Slider--Ground";
        }



/**testing zeroing of Intake slider zero position*****/
        if(gamepad2.dpad_up) {
            robot.Intake.intakeSlideOUT();
            robot.Intake.intakeDOWN();
            intakeOptionInitLoop = "Intake Slider--OUT";
        }
        if (gamepad2.dpad_down) {
            robot.Intake.intakeSlideIN();
            robot.Intake.intakeTRANSFER();
            intakeOptionInitLoop = "Intake Slider--IN, zero";
        }

        //TODO: if right outtake slider is <10 position and
        //      angular velocity is <0.02 (1 degree = 0.175 radian), then reset slider to ZERO position, need to test and adjust
        if ((intakeSliderPresentPosition < 20) && (Math.abs(intakeSliderAngularVelocity)<1)){
            robot.Intake.intakeSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //robot.Intake.intakeSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //TODO: do we need this line?
            intakeOptionInitLoop = "reset Intake Slider to zero";
        }

        telemetryA.addLine(outtakeOptionInitLoop);
        telemetryA.addLine(intakeOptionInitLoop);














//        robot.LED.LEDinitReady();
//
//        telemetryA.setMsTransmissionInterval(50);
//        telemetryA.addData("Battery Voltage (V): ", "%.1f", battery.getVoltage());
//        telemetryA.addData("Bot Heading--imu Yaw (degrees): ", "%.1f", botHeadingImu);
//        telemetryA.addData("Robot Driving Orientation if not PedroPath = ", drivingOrientation);
//        telemetryA.addLine("");
//        telemetryA.addData("Outtake left slide position: ", robot.Outtake.outtakeRightSlide.getCurrentPosition());
//        telemetryA.addData("Outtake right slide position: ",  robot.Outtake.outtakeLeftSlide.getCurrentPosition());
//        telemetryA.addLine("");
//        telemetryA.addLine("current check--current spike if stalling");
//        //when stalling/spike is detected, it means the slide is at lowest or max position and so can reset it as zero position
//        telemetryA.addData("Outtake left slide motor current (mA): ", "%.1f", robot.Outtake.outtakeRightSlide.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetryA.addData("Outtake right slide motor current (mA): ",  "%.1f", robot.Outtake.outtakeLeftSlide.getCurrent(CurrentUnit.MILLIAMPS));
//
//
//        telemetryA.addData("red", robot.Sensor.colorTest.red());
//        telemetryA.addData("hue", robot.Sensor.getColorInfo(0));
//        telemetryA.addLine();

        telemetryA.addData("Runtime (seconds) = ", "%.1f", getRuntime());
//        telemetryA.addData("Distance Sensor = ", "%.1f", robot.Sensor.getDistance());
        telemetryAllColorInfo();
        gamePadColorControl();
        robot.LED.ledStick.setColor(5, Color.WHITE);
        robot.LED.ledStick.setBrightness(0,0);
        robot.LED.ledStick.setBrightness(1,0);
        robot.LED.ledStick.setBrightness(2,0);
        robot.LED.ledStick.setBrightness(3,0);
        robot.LED.ledStick.setBrightness(4,0);
        robot.LED.ledStick.setBrightness(5,1);
        robot.LED.ledStick.setBrightness(6,0);
        robot.LED.ledStick.setBrightness(7,0);
        robot.LED.ledStick.setBrightness(8,0);
        robot.LED.ledStick.setBrightness(9,0);
        robot.LED.ledStick.setBrightness(10,0);

        //PedroPathing coordiante
        telemetryA.addData("X PedroPathing = ","%.1f", follower.getPose().getX());
        telemetryA.addData("Y PedroPathing = ","%.1f", follower.getPose().getY());
        telemetryA.addData("Heading PedroPathing (Deg) = ","%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetryA.addData("Bot Heading--imu Yaw (deg) = ", "%.1f", botHeadingImu);
        telemetryA.addLine(" ");
        //telemetryA.addData("X from RR ", follower.getPose().getX());
        //telemetryA.addData("Y from RR ", follower.getPose().getY());
        //telemetryA.addData("Bot Heading from RR (degrees) ", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        //telemetryA.addLine("");
        //telemetryA.addData("Bot Heading--imu Yaw (degrees)", "%.1f", botHeadingImu);

        loopTimeTotal = loopTimeTotal + loopTime.milliseconds();
        loopTimeCount = loopTimeCount + 1;
        loopTimeAverMilliSec = loopTimeTotal/loopTimeCount;
        writeDatalog();             //TODO: check on effect of loopTimeAverMilliSec with and without this datalogging
        telemetryA.addData("Average loop Time (ms) = ", "%.3f", loopTimeAverMilliSec);


        telemetrySubsystemStatus();
        telemetryA.update();

    }

    //-------------------------------------------------------------------------------------------------
    @Override
    public void start() {
        robot.start();
        dataTimer.reset();  // Reset timer for datalogging interval.
        datalogOpModeStatus = "RUNNING";

        runtime.reset();

        drivingOrientation = "robotOriented";
        state = State.START;
        robot.LED.ledStick.setColor(5, Color.WHITE);
        robot.LED.ledStick.setBrightness(0,0);
        robot.LED.ledStick.setBrightness(1,0);
        robot.LED.ledStick.setBrightness(2,0);
        robot.LED.ledStick.setBrightness(3,0);
        robot.LED.ledStick.setBrightness(4,0);
        robot.LED.ledStick.setBrightness(5,1);
        robot.LED.ledStick.setBrightness(6,0);
        robot.LED.ledStick.setBrightness(7,0);
        robot.LED.ledStick.setBrightness(8,0);
        robot.LED.ledStick.setBrightness(9,0);
        robot.LED.ledStick.setBrightness(10,0);

        //This starts teleop drive control by 1. breakFollowing() and set teleopDrive = true;
        //If regular manual control by JAVA for FTC method, then comment this out.
//        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        bulkReadTELEOP();
        botHeadingImu = imuAngle;
        loopTime.reset();
        //TODO: if right outtake slider is <10 position and
        //      angular velocity is <1 tick/second (1 degree = 0.175 radian), then reset slider to ZERO position
        if ((outtakeSlider_Right_PresentPosition < 10) &&
                (Math.abs(outtakeSlider_Right_AngularVelocity)<1)
                && (isGP2rightYupdown || isGP2leftYupdown)){
            //!((Math.abs(gamepad2.right_stick_y) > 0.0)) || (Math.abs(gamepad2.left_stick_y) > 0.0)){
            //){
            robot.Outtake.outtakeLeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.Outtake.outtakeRightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //robot.Outtake.outtakeLeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  //TODO: do we need this line?
            //robot.Outtake.outtakeRightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //TODO: do we need this line?
            //outtakeOptionInitLoop = "reset Outtake Slider to zero";
        }
        //TODO: if right outtake slider is <10 position and
        //      angular velocity is <1 tick/second (1 degree = 0.175 radian), then reset slider to ZERO position, need to test and adjust
//        if ((intakeSliderPresentPosition < 1) && (Math.abs(intakeSliderAngularVelocity)<1)){
//            robot.Intake.intakeSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            //robot.Intake.intakeSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); //TODO: do we need this line?
//            //intakeOptionInitLoop = "reset Intake Slider to zero";
//        }



        switch (state) {
            case START:
                telemetryA.addLine("CASE: Start");
                telemetryA.addData("START Outtake Arm Position: ",robot.Outtake.getOuttakeArmPosition());
                if(gamepad2.a || outtakeOption.equals("start")) {
                    robot.Outtake.readyPosition();
                    robot.Intake.intakeTRANSFER();
                    robot.Intake.sweeperIN();
                    outtakeOption = "";
                }
                if (gamepad1.y) {
                    outtakeOption = "hang";
                }
                if (outtakeOption.equals("hang")) {
                    robot.Outtake.closeClaw();
                    robot.Intake.intakeUP();
                    robot.Outtake.outtakeArmAxon.setPosition(0.2);
                    robot.Outtake.extendOUT();
                    robot.Hang.hangServo.setPosition(0.09);//closed position is 1.0
                }
//                if(gamepad2.dpad_down){
//                    outtakeOption = "wallIntakeBack";
//                    //state = State.OUTTAKE_READY;
//                }
                if(gamepad2.dpad_down){
                    outtakeOption = "wallIntakeFront";
                    state = State.OUTTAKE;
                }
                if(gamepad1.dpad_up){
                    robot.Intake.intakeSlideOUT();
                    robot.Intake.intakeUP();
                    state = State.INTAKE;
                }
                else if(gamepad1.dpad_left){
                    robot.Intake.intakeSlideMID();
                    robot.Intake.intakeUP();
                    state = State.INTAKE;
                }
                else if(gamepad1.dpad_right){
                    robot.Intake.intakeSlideLOW();
                    robot.Intake.intakeUP();
                    state = State.INTAKE;
                }
                else if(gamepad1.dpad_down){
                    robot.Intake.intakeSTOP();
                    robot.Intake.intakeTRANSFER();
                    robot.Intake.intakeSlideIN();
                }
                if (robot.Intake.intakeSlides.getCurrentPosition() < 10 && robot.Intake.intakeServoAxon.getPosition() == 1 && gamepad2.right_trigger > 0.2) { //TODO: Find correct servo position
                    state = State.TRANSFER;
                }
                break;
            case INTAKE:
                //telemetryAllColorInfo();
//                if (sampleColor.equals("NONE")){
//                    robot.LED.ledStick.setColor(1, Color.WHITE);
//                    robot.LED.ledStick.setBrightness(1,1);
//                    robot.LED.ledStick.setBrightness(2,0);
//                    robot.LED.ledStick.setBrightness(3,0);
//                    robot.LED.ledStick.setBrightness(4,0);
//                    robot.LED.ledStick.setBrightness(5,0);
//                    robot.LED.ledStick.setBrightness(6,0);
//                    robot.LED.ledStick.setBrightness(7,0);
//                    robot.LED.ledStick.setBrightness(8,0);
//                    robot.LED.ledStick.setBrightness(9,0);
//                    robot.LED.ledStick.setBrightness(10,0);
//                }
                if (gamepad1.left_trigger > 0.2 && robot.Intake.intakeServoAxon.getPosition() > 0.8){
                    robot.Intake.intakeDOWN();
                    robot.Intake.intakeIN();
                }
                else if (gamepad1.left_trigger > 0.2 && robot.Intake.intakeServoAxon.getPosition() < 0.7){
                    robot.Intake.intakeIN();
                }
                else if (gamepad1.left_bumper || sampleColor.equals(nonAllianceColor)){
//                    if (sampleColor.equals("RED")){
//                        robot.LED.ledStick.setColor(Color.RED);
//                        robot.LED.ledStick.setBrightness(1);
//                    }
                    robot.Intake.intakeUP();
                    robot.Intake.intakeOUT();
                }
                else{
                    robot.Intake.intakeUP();
                    robot.Intake.intakeSTOP();

                }
                if(gamepad1.dpad_up){
                    robot.Intake.intakeSlideOUT();
                }
                else if(gamepad1.dpad_left){
                    robot.Intake.intakeSlideMID();
                }
                else if(gamepad1.dpad_right || (!intakeExtend && (sampleColor.equals(allianceColor) || sampleColor.equals("YELLOW")))){
//                    if (sampleColor.equals("BLUE")){
//                        robot.LED.ledStick.setColor(Color.BLUE);
//                        robot.LED.ledStick.setBrightness(1);
//                    }
//                    else if (sampleColor.equals("YELLOW")){
//                        robot.LED.ledStick.setColor(Color.YELLOW);
//                        robot.LED.ledStick.setBrightness(1);
//                    }
                    robot.Intake.intakeSlideLOW();
                    if(gamepad1.dpad_down){
                        state = State.TRANSFER;
                        robot.Intake.intakeSTOP();
                        robot.Intake.intakeTRANSFER();
                        robot.Intake.intakeSlideIN();
                    }
                    if (gamepad1.ps){
                        intakeExtend = true;
                    }
                }
                else if(gamepad1.dpad_down){
                    state = State.TRANSFER;
                    robot.Intake.intakeSTOP();
                    robot.Intake.intakeTRANSFER();
                    robot.Intake.intakeSlideIN();
                }
                break;
            case TRANSFER:
                intakeExtend = false;
//                telemetryA.addData("Outtake Arm Position actual reading:",robot.Outtake.getOuttakeArmPosition());
//                telemetryA.addData("Outtake Arm set position:", robot.Outtake.outtakeArmAxon.getPosition());
//                telemetryA.addData("Intake Axon Servo Position actual reading:",robot.Intake.getIntakeServoAxonPosition());
//                telemetryA.addData("Intake Axon Servo set position:", robot.Intake.getIntakeServoAxonPosition());
                if (/*robot.Outtake.getOuttakeArmPosition() < 0.4 && robot.Outtake.getOuttakeArmPosition() > 0.32 &&*/ robot.Outtake.outtakeArmAxon.getPosition() < 0.35 && gamepad2.left_trigger > 0.2) {
                    robot.Outtake.closeClaw();
                }
                else if(gamepad2.left_bumper){
                    robot.Outtake.openClaw();
                }
                if (gamepad1.left_trigger > 0.2 && robot.Intake.intakeServoAxon.getPosition() < 0.7){
                    robot.Intake.intakeIN();
                }
                else {
                    robot.Intake.intakeSTOP();
                }

                if (gamepad2.left_trigger > 0.2){
                    robot.Outtake.groundPosition();
                }
//                else if (robot.Intake.getIntakeServoAxonPosition() < 0.4){ //TODO: Find correct axon value
//                    robot.Outtake.groundPosition();
//                }

                if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right){ //Be able to intake again
                    state = State.INTAKE;
                }
                if(gamepad2.a || outtakeOption.equals("start") || gamepad2.left_bumper) { // this current code would be in transfer state when intake ready
                    robot.Outtake.readyPosition();
                    outtakeOption = "";
                }
//                else if(gamepad2.left_trigger > 0.2 || gamepad2.right_trigger > 0.2){
//                    robot.Outtake.groundPositionClose();
//                }
                if(robot.Outtake.claw.getPosition() > 0.27 && gamepad2.y) { //Make sure that claw is in closed position
                    //robot.Outtake.extendIN();
                    outtakeOption = "highBasket";
                    //robot.Outtake.closeClaw();
                    state = State.OUTTAKE;
                }
                if(robot.Outtake.claw.getPosition() > 0.27 && gamepad2.dpad_left) { //Make sure that claw is in closed position
                    //robot.Outtake.extendIN();
                    outtakeOption = "lowBasket";
                    //robot.Outtake.closeClaw();
                    state = State.OUTTAKE;
                }
                if(gamepad2.dpad_down){
                    outtakeOption = "wallIntakeFront";
                    state = State.OUTTAKE;
                }
                if(gamepad2.dpad_right && robot.Outtake.claw.getPosition() > 0.2){
//                    outtakeOption = "sampleDeliver";
                    state = State.OUTTAKE;
                }
                break;
            case OUTTAKE_READY:
                robot.Outtake.readyPosition();
                if(outtakeOption.equals("highBasket") && robot.Outtake.outtakeLeftSlide.getCurrentPosition()>0){
                    state = State.OUTTAKE;
                }
                if(outtakeOption.equals("wallIntake") && robot.Outtake.outtakeLeftSlide.getCurrentPosition()>400){
                    state = State.OUTTAKE;
                }
                break;

            case OUTTAKE:
//                telemetryA.addData("Outtake Arm Position actual reading:",robot.Outtake.getOuttakeArmPosition());
//                telemetryA.addData("Outtake Arm set position:", robot.Outtake.outtakeArmAxon.getPosition());
//                telemetryA.addData("Outtake claw Position: ",robot.Outtake.claw.getPosition());
//                telemetryA.addData("Outtake left slide Position: ",robot.Outtake.outtakeLeftSlide.getCurrentPosition());
                if (outtakeOption.equals("highBasket")){
                    robot.Outtake.leftSlideSetPositionPower(875,1);
                    robot.Outtake.rightSlideSetPositionPower(875,1);
                    if (robot.Outtake.outtakeLeftSlide.getCurrentPosition()>200){
                        robot.Outtake.extendIN();
                        robot.Outtake.highBasket();
                    }
                }
                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 700 && gamepad2.left_bumper){ // If at high basket position
                    robot.Outtake.openClaw();
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 700 && gamepad2.a && robot.Outtake.claw.getPosition() < 0.05){ // Should robot make sure claw is open before going down
                    outtakeOption = "start";
                    state = State.START;
                }
                if (outtakeOption.equals("lowBasket")){
                    robot.Outtake.lowBasket();
                }
                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 200 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 350 && gamepad2.left_bumper){ // If at high basket position
                    robot.Outtake.openClaw();
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 200 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 350 && gamepad2.a && robot.Outtake.claw.getPosition() < 0.05){ // Should robot make sure claw is open before going down
                    outtakeOption = "start";
                    state = State.START;
                }


                if (outtakeOption.equals("sampleDeliver")) { //Deliver sample to human player
                    robot.Intake.intakeOUT();
                    robot.Outtake.sampleDelivery();
                    if (robot.Outtake.getOuttakeArmPosition() > 0.8){
                        robot.Outtake.openClaw();
                        robot.Intake.intakeSTOP();
                        state = State.READY_DOWN;
                    }
                    else{
                        robot.Intake.intakeOUT();
                    }
                }

                if (outtakeOption.equals("wallIntakeFront")){
                    if (gamepad2.a){
                        state = State.READY_DOWN;
                    }
                    else {
                        if (robot.Outtake.getOuttakeArmPosition() > 0.7){
                            robot.Intake.intakeINSIDEBOT();
                            robot.Outtake.leftSlideSetPositionPower(0,1);
                            robot.Outtake.rightSlideSetPositionPower(0,1);
                            robot.Outtake.outtakeArmAxon.setPosition(0.28);
                        }
                        else {
                            robot.Intake.intakeINSIDEBOT();
                            robot.Outtake.openClaw();
                            robot.Outtake.wallIntakeFront();
                        }
                    }
                }
                if (robot.Outtake.outtakeArmAxon.getPosition() < 0.35  && gamepad2.left_trigger > 0.2){ // Only if at wall intake position
                    robot.Outtake.closeClaw();
                    outtakeOption = "";
                }
                else if (robot.Outtake.outtakeArmAxon.getPosition() < 0.3  && gamepad2.left_bumper){
                    robot.Outtake.openClaw();
                }
                if (gamepad2.x){
                    robot.Outtake.leftSlideSetPositionPower(475,1);
                    robot.Outtake.rightSlideSetPositionPower(475,1);
                    outtakeOption = "highChamber";
                }

                if (outtakeOption.equals("highChamber") && robot.Outtake.outtakeLeftSlide.getCurrentPosition()>5){
                    robot.Outtake.highChamberSetUpwards();
                }
                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 450 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 525 && (gamepad2.left_bumper)){ // Open claw if specimen scored
                    robot.Outtake.openClaw();
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 450 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 525 && robot.Outtake.claw.getPosition() < 0.05 && gamepad2.a){ // Only if at high Chamber set position and claw is open
                    state = State.READY_DOWN;
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 450 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 525 && robot.Outtake.claw.getPosition() < 0.05 && gamepad2.dpad_down){ // Only if at high Chamber set position and claw is open
                    outtakeOption = "wallIntakeFront";
                }
                //Option to score specimen by moving slides up
                if (robot.Outtake.outtakeArmAxon.getPosition() > 0.85 && gamepad2.b){ // Only if at high chamber set position
                    outtakeOption = "highChamberFinish";
                }
                if (outtakeOption.equals("highChamberFinish")){
                    robot.Outtake.highChamberFinishUpwards();
                }
                if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 525 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 700 && gamepad2.left_bumper){ // If at high chamber finish position
                    robot.Outtake.openClaw();
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 525 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 700 && robot.Outtake.claw.getPosition() < 0.05 && gamepad2.a){ // Only if at high Chamber finish position and claw is open
                    state = State.READY_DOWN;
                }
                else if (robot.Outtake.outtakeLeftSlide.getCurrentPosition() > 525 && robot.Outtake.outtakeLeftSlide.getCurrentPosition() < 700 && robot.Outtake.claw.getPosition() < 0.05 && gamepad2.dpad_down){ // Only if at high Chamber finish position and claw is open
                    outtakeOption = "wallIntakeFront";
                }
                if (gamepad1.y)
                {
                    state = State.START;
                }
                break;
            case READY_DOWN:
                robot.Outtake.readyPosition();
                if(robot.Outtake.outtakeLeftSlide.getCurrentPosition()<600){
                    outtakeOption = "start";
                    state = State.START;
                }
                break;
        }


        //Hang testing:
        // Move slides up before hanging
        if (gamepad1.y) {
            robot.Hang.hangServo.setPosition(0.05);
        }
        // Pull slides down to hang
        else if(gamepad1.x) {
            robot.Hang.hang();
        }


        //***MANUAL SLIDES
        if (gamepad2.right_stick_y > 0.1 || gamepad2.left_stick_y > 0.1){
            int currentLeftSlidePosition = robot.Outtake.outtakeLeftSlide.getCurrentPosition();
            int currentRightSlidePosition = robot.Outtake.outtakeRightSlide.getCurrentPosition();
            robot.Outtake.outtakeLeftSlide.setTargetPosition(currentLeftSlidePosition + 50);
            robot.Outtake.outtakeRightSlide.setTargetPosition(currentRightSlidePosition + 50);
            boolean isGP2rightYupdown = true;
            boolean isGP2leftYupdown = true;
        }
        if (gamepad2.right_stick_y  < -0.1 || gamepad2.left_stick_y  < -0.1){
            int currentLeftSlidePosition = robot.Outtake.outtakeLeftSlide.getCurrentPosition();
            int currentRightSlidePosition = robot.Outtake.outtakeRightSlide.getCurrentPosition();
            robot.Outtake.outtakeLeftSlide.setTargetPosition(currentLeftSlidePosition - 50);
            robot.Outtake.outtakeRightSlide.setTargetPosition(currentRightSlidePosition - 50);
            boolean isGP2rightYupdown = true;
            boolean isGP2leftYupdown = true;
        }
        if (gamepad2.ps){
            robot.Outtake.outtakeLeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.Outtake.outtakeRightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.Outtake.outtakeLeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.Outtake.outtakeRightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        isGP2rightYupdown = false;
        isGP2leftYupdown = false;

//        if (sampleColor.equals("YELLOW")){
//            robot.LED.ledStick.setColor(Color.YELLOW);
//            robot.LED.ledStick.setBrightness(1);
//        }else if (sampleColor.equals("BLUE")){
//            robot.LED.ledStick.setColor(Color.BLUE);
//            robot.LED.ledStick.setBrightness(1);
//        }else if (sampleColor.equals("RED")){
//            robot.LED.ledStick.setColor(Color.RED);
//            robot.LED.ledStick.setBrightness(1);
//        }else{
//            robot.LED.ledStick.setColor(5, Color.WHITE);
//            robot.LED.ledStick.setBrightness(0,0);
//            robot.LED.ledStick.setBrightness(1,0);
//            robot.LED.ledStick.setBrightness(2,0);
//            robot.LED.ledStick.setBrightness(3,0);
//            robot.LED.ledStick.setBrightness(4,0);
//            robot.LED.ledStick.setBrightness(5,1);
//            robot.LED.ledStick.setBrightness(6,0);
//            robot.LED.ledStick.setBrightness(7,0);
//            robot.LED.ledStick.setBrightness(8,0);
//            robot.LED.ledStick.setBrightness(9,0);
//            robot.LED.ledStick.setBrightness(10,0);
//        }


        if (gamepad2.right_trigger > 0.2){
            robot.Intake.sweeperOUT();
        }
        if (gamepad2.right_trigger <= 0.2){
            robot.Intake.sweeperIN();
        }


//Drivetrain Movement:
//MANUAL DRIVE for Mecanum wheel drive.
        y = -gamepad1.left_stick_y;           // Remember,joystick value is reversed!
        x = -gamepad1.left_stick_x;             // this is strafing  V1=positive
        rx = -gamepad1.right_stick_x;                // this is strafing  V1=positive

        //Cancel angle movement of gamepad left stick, make move move either up/down or right/left
        //if (Math.abs(y) >= Math.abs(x)) {
        //  y = y;
        //  x = 0;
        //  } else {
        //      y = 0;
        //      x = x;
        //  }
        //DRIVETRAIN
        //baseline speed =  reduce motor speed to 60% max
        //double motorPowerDefault = 0.5;
        double powerChange;

        //SLOW DOWN with RIGHT LOWER TRIGGER (lower of the top side button) press with the other gamepad stick.
        if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_trigger > 0.1)) {
            powerShift  = 0.5;
            //SPEED UP with RIGHT UPPER BUMPER (up of the top side button) press with the other gamepad stick.
        } else if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_bumper) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_bumper) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_bumper)) {
            powerShift  = 1.0;
        } else {
            powerShift  = motorPowerDefault;
        }
        //powerShift = motorPowerDefault + powerChange;

/**12/18/2024--THIS IS COMMENTED OUT WHEN USING PEDROPATHING TO DRIVE ROBOT
 drivetrain.drive(y, x, rx, powerShift, botHeadingImu, drivingOrientation);
 */
/** Comment out if using regular drivetrain to drive robot, otherwise use below.
 * Update Pedro to move the robot based on:
 - Forward/Backward Movement: -gamepad1.left_stick_y
 - Left/Right Movement: -gamepad1.left_stick_x
 - Turn Left/Right Movement: -gamepad1.right_stick_x
 - Robot-Centric Mode: true
 --original for PedroPathing:  follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
 */
        //follower.setTeleOpMovementVectors(-y*powerShift, -x*powerShift, -rx*powerShift, true);
        //Below set the max power as set above
        follower.setTeleOpMovementVectors(
                Range.clip(y,-powerShift, +powerShift),
                Range.clip(x,-powerShift, +powerShift),
                Range.clip(rx,-powerShift, +powerShift), true);
        follower.update();



        /**************  TELEMETRY MAY SLOW DOWN LOOP if many I2C calls  ************************/
        telemetryA.addData("Runtime (seconds) = ", "%.1f", getRuntime());
        telemetryA.addData("Hang motor position: ", robot.Hang.hangMotor.getCurrentPosition());
        //telemetryA.addData("Robot Driving Orientation = ", drivingOrientation);
        //  telemetryA.addData("State = ", state);
        //telemetryA.addData("Time in State (seconds) = ", 0);
        //telemetryA.addData("lastTime = ", lastTime);
        //telemetryA.addLine(" ");
        telemetryA.addData("1OuttakeSlider LEFT Current mA = ", robot.Outtake.getOuttakeSliderLeftCurrent());
        telemetryA.addData("1Outtake LEFT slide Pos: ", robot.Outtake.outtakeLeftSlide.getCurrentPosition());
        telemetryA.addLine(" ");
        telemetryA.addData("2OuttakeSlider RIGHT Current mA = ", robot.Outtake.getOuttakeSliderRightCurrent());
        telemetryA.addData("3Outtake RIGHT slide Pos: ",robot.Outtake.outtakeRightSlide.getCurrentPosition());
        telemetryA.addLine(" ");
        telemetryA.addData("5Outtake Axon Arm actual Pos: ",robot.Outtake.getOuttakeArmPosition());
        telemetryA.addData("6Outtake Axon Arm set Pos: ", robot.Outtake.outtakeArmAxon.getPosition());
        telemetryA.addData("7Outtake claw Pos: ", robot.Outtake.claw.getPosition());
        telemetryA.addLine(" ");
        telemetryA.addData("8Intake slider Current mA = ", robot.Intake.getIntakeSlideCurrent());
        telemetryA.addData("9Intake slider Pos: ", robot.Intake.intakeSlides.getCurrentPosition());
        telemetryA.addLine(" ");
        telemetryA.addData("Intake Axon actual Pos: ",robot.Intake.getIntakeServoAxonPosition());
        telemetryA.addData("Intake Axon set Pos: ", robot.Intake.getIntakeServoAxonPosition());
        telemetryA.addLine(" ");

//        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
//        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
//        Drawing.sendPacket();


//        if ((runtime.seconds() > 75) && endGameRumble45secondsWarningOnce) {
//            rumble();
//            telemetryA.addLine("45 SECONDS LEFT");
//            telemetryA.addLine("");
//            endGameRumble45secondsWarningOnce = false;
//            gamepad1.runLedEffect(flashingWhite6Sec);
//        }
//        if ((runtime.seconds()  > 89) && endGameRumble31secondSTARTonce) {
//            rumble();
//            telemetryA.addLine("30 SECONDS LEFT");
//            telemetryA.addLine("");
//            rumble();
//            endGameRumble31secondSTARTonce = false;
//            gamepad1.runLedEffect(flashingBlue6Sec);
//        }

        //one rumble for 16 seconds left
        if ((runtime.seconds() > 104 ) && endGameRumble16secondsLeftOnce) {
            rumble();
            telemetryA.addLine("16 SECONDS LEFT");
            telemetryA.addLine("");
            endGameRumble16secondsLeftOnce = false;
            gamepad1.runLedEffect(flashingRed6Sec);
        }

        //3 rumble for 8 seconds left for hanging
        if ((runtime.seconds() > 112 ) && endGameRumble8secondsLeftOnce) {
            rumble6sec();
            telemetryA.addLine("10 SECONDS LEFT---HANGING NOW");
            telemetryA.addLine("");
            endGameRumble16secondsLeftOnce = false;
            gamepad1.runLedEffect(flashingRed6Sec);
        }

        loopTimeTotal = loopTimeTotal + loopTime.milliseconds();
        loopTimeCount = loopTimeCount + 1;
        loopTimeAverMilliSec = loopTimeTotal/loopTimeCount;
        telemetryA.addData("Average loop Time (ms) = ", "%.1f", loopTimeAverMilliSec);

        writeDatalog();         //TODO: if having Datalogging take too much loop lime (test with init loop above),then remove
        telemetryA.update();

    }


    @Override
    public void stop() {
        robot.stop();
        drivetrain.setMotorPower(0, 0, 0, 0);
        datalog.closeDataLogger();
    }


    public void bulkReadTELEOP() {
        imuAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);       //0. direction is reverse
        batteryStatus = battery.getVoltage();
        //newRightEncoder = robot.RBack_Motor.getCurrentPosition();       //1.
        intakeSliderPresentPosition = robot.Intake.intakeSlides.getCurrentPosition();
        intakeSliderCurrentDraw = robot.Intake.intakeSlides.getCurrent(CurrentUnit.MILLIAMPS);
        intakeSliderAngularVelocity = robot.Intake.intakeSlides.getVelocity();   //TODO: what is the unit
        outtakeSlider_Right_PresentPosition = robot.Outtake.outtakeRightSlide.getCurrentPosition();
        outtakeSlider_Right_CurrentDraw = robot.Outtake.outtakeRightSlide.getCurrent(CurrentUnit.MILLIAMPS);
        outtakeSlider_Right_AngularVelocity = robot.Outtake.outtakeRightSlide.getVelocity();   //TODO: what is the unit

    }
    public void telemetrySubsystemStatus(){
        telemetryA.addData("1intakeSliderCurrent mA: ", intakeSliderCurrentDraw);
        telemetryA.addData("1intakeSliderPosition: ", intakeSliderPresentPosition);
        telemetryA.addData("1intakeSliderAnVel Deg: ", Math.toDegrees(intakeSliderAngularVelocity));
        telemetryA.addLine(" ");
        telemetryA.addData("2outtakeSlider_R_Current mA: ", outtakeSlider_Right_CurrentDraw);
        telemetryA.addData("2outtakeSlider_R_Position: ", outtakeSlider_Right_PresentPosition);
        telemetryA.addData("2outtakeSlider_R_AngVel Deg: ", Math.toDegrees(outtakeSlider_Right_AngularVelocity));
        telemetryA.addLine(" ");
    }







    public void writeDatalog() {
        if (dataTimer.time() > logInterval) {
            readCount++;
            // Populate the fields to be logged.  Must appear in the same order
            // as the column headings, to ensure data is logged to the intended
            // field name.
            datalog.addField(readCount);
            datalog.addField(datalogOpModeStatus);
            datalog.addField(getRuntime());
            datalog.addField(batteryStatus);
            datalog.addField(imuAngle);
            datalog.addField(loopTimeAverMilliSec);
            datalog.newLine();                // end the current set of readings
            dataTimer.reset();      // start the interval timer again

        }   // end if(timer)
    }




    public void telemetryAllColorInfo(){
//        telemetryA.addData("Gain", colorGain);
//        telemetryA.addLine("");
//        // Tell sensor desired gain value (normally you would do this during initialization, not during loop)
//        robot.Sensor.colorIntake.setGain(colorGain);
        // Get the normalized colors from the sensor

        NormalizedRGBA colors = robot.Sensor.colorIntake.getNormalizedColors();
        double blue = colors.blue;
        double red = colors.red;
        double green = colors.green;
        if (red > 0.02 && red > green && red > blue){
            sampleColor = "RED";
            robot.LED.ledStick.setColor(Color.RED);
            robot.LED.ledStick.setBrightness(1);
        }
        else if (blue > 0.02 && blue > green && blue > red){
            sampleColor = "BLUE";
            robot.LED.ledStick.setColor(Color.BLUE);
            robot.LED.ledStick.setBrightness(1);
        }
        else if (green > 0.02 && green > red && green > blue){
            sampleColor = "YELLOW";
            robot.LED.ledStick.setColor(Color.YELLOW);
            robot.LED.ledStick.setBrightness(1);
        }
        else{
            sampleColor = "NONE";
            robot.LED.ledStick.setColor(5, Color.WHITE);
            robot.LED.ledStick.setBrightness(0,0);
            robot.LED.ledStick.setBrightness(1,0);
            robot.LED.ledStick.setBrightness(2,0);
            robot.LED.ledStick.setBrightness(3,0);
            robot.LED.ledStick.setBrightness(4,0);
            robot.LED.ledStick.setBrightness(5,1);
            robot.LED.ledStick.setBrightness(6,0);
            robot.LED.ledStick.setBrightness(7,0);
            robot.LED.ledStick.setBrightness(8,0);
            robot.LED.ledStick.setBrightness(9,0);
            robot.LED.ledStick.setBrightness(10,0);
        }
        /* Use telemetry to display feedback on Driver Station. Show red/green/blue normalized values
         *from sensor (0 to 1), and equivalent HSV (hue/saturation/value) values.
         * See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for explanation of HSV color. */
        // Update the hsvValues array by passing it to Color.colorToHSV()
//        Color.colorToHSV(colors.toColor(), hsvValues);
//        telemetryA.addData("Red", "%.3f", colors.red);
//        telemetryA.addData("Green", "%.3f", colors.green);
//        telemetryA.addData("Blue", "%.3f", colors.blue);
//        telemetryA.addLine("");
//        telemetryA.addData("Hue", "%.3f", hsvValues[0]);
//        telemetryA.addData("Saturation", "%.3f", hsvValues[1]);
//        telemetryA.addData("Value", "%.3f", hsvValues[2]);
//        telemetryA.addLine("");
//        telemetryA.addData("Alpha", "%.3f", colors.alpha);
//        telemetryA.addLine("");
        telemetryA.addLine("Sample color: " + sampleColor);
        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
//        if (robot.Sensor.colorIntake instanceof DistanceSensor) {
//            telemetryA.addData("Distance (cm)", "%.3f", ((DistanceSensor) robot.Sensor.colorIntake).getDistance(DistanceUnit.CM));
//        }
    }

    public void gamePadColorControl(){
        telemetryA.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetryA.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Update the gain value if either of the A or B gamepad buttons is being held
        if (gamepad1.a) {
            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
            colorGain += 0.5;
        } else if (gamepad1.b && colorGain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
            colorGain -= 0.005;
        }
        // Check the status of the X button on the gamepad
        xButtonCurrentlyPressed = gamepad1.x;

        // If the button state is different than what it was, then act
        if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
            // If the button is (now) down, then toggle the light
            if (xButtonCurrentlyPressed) {
                if (robot.Sensor.colorIntake instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight)robot.Sensor.colorIntake;
                    light.enableLight(!light.isLightOn());
                }
            }
        }
        xButtonPreviouslyPressed = xButtonCurrentlyPressed;

    }
    public void rumble(){
        Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
        gamepad1.runRumbleEffect(customRumbleEffect);
        gamepad2.runRumbleEffect(customRumbleEffect);
    }

    public void rumble6sec(){
        Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec

                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec

                .addStep(0.0, 1.0, 100)  //  Rumble right motor 100% for 500 mSec
                .addStep(1.0, 0.0, 100)  //  Rumble left motor 100% for 250 mSec
                .build();
        gamepad1.runRumbleEffect(customRumbleEffect);
        gamepad2.runRumbleEffect(customRumbleEffect);
    }




}