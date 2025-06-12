import android.graphics.Color;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

import Diagnostic.Datalogger;
import Hardware.HardwareDrivetrain;
import Hardware.HardwareNoDriveTrainRobot;
import RR.AUTOstorageConstant;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

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
@TeleOp(group="Primary", name= "Demo_TeleOp")

public class Demo_TeleOp extends OpMode {
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
        motorPowerDefault = 0.3;


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

    }


    @Override
    public void init_loop() {
        loopTime.reset();
        botHeadingImu = imuAngle;
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

        //This starts teleop drive control by 1. breakFollowing() and set teleopDrive = true;
        //If regular manual control by JAVA for FTC method, then comment this out.
//        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        robot.Outtake.readyPosition();
        robot.Intake.intakeTRANSFER();
        robot.Intake.sweeperIN();






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
//        if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_trigger > 0.1) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_trigger > 0.1)) {
//            powerShift  = 0.5;
//            //SPEED UP with RIGHT UPPER BUMPER (up of the top side button) press with the other gamepad stick.
//        } else if ((Math.abs(gamepad1.left_stick_y) > 0.1 && gamepad1.right_bumper) || (Math.abs(gamepad1.left_stick_x) > 0.1 && gamepad1.right_bumper) || (Math.abs(gamepad1.right_stick_x) > 0.1 && gamepad1.right_bumper)) {
//            powerShift  = 1.0;
//        } else {
//            powerShift  = motorPowerDefault;
//        }
        powerShift = motorPowerDefault;

        follower.setTeleOpMovementVectors(
                Range.clip(y,-powerShift, +powerShift),
                Range.clip(x,-powerShift, +powerShift),
                Range.clip(rx,-powerShift, +powerShift), true);
        follower.update();

    }


    @Override
    public void stop() {
        robot.stop();
        drivetrain.setMotorPower(0, 0, 0, 0);
        datalog.closeDataLogger();
    }
}