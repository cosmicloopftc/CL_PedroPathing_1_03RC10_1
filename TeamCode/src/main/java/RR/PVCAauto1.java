package RR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.HardwareNoDriveTrainRobot;
import RR.tuning.TuningOpModes;

@Config
@Autonomous(name = "PVCAautoV1", group = "Auto")
public final class PVCAauto1 extends LinearOpMode {

    HardwareNoDriveTrainRobot autoRobot = new HardwareNoDriveTrainRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        Telemetry telemetryA;
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //adjust these settings as needed for use in the trajectory codes
        VelConstraint velSlow = new TranslationalVelConstraint(15);
        VelConstraint velMediumMax = new TranslationalVelConstraint(60);
        VelConstraint velFastMin = new TranslationalVelConstraint(30);  // prev 40
        VelConstraint velMedium = new TranslationalVelConstraint(30);
        VelConstraint velMediumPush = new TranslationalVelConstraint(32);
        VelConstraint velFast = new TranslationalVelConstraint(70);  // prev 105, 70
        VelConstraint velStupidFast = new TranslationalVelConstraint(200);
        AccelConstraint accSlow = new ProfileAccelConstraint(-15,15);
        AccelConstraint accMedium = new ProfileAccelConstraint(-50,50);  // prev 55
        AccelConstraint accFastMax = new ProfileAccelConstraint(-80,80);
        AccelConstraint accFastMin = new ProfileAccelConstraint(-70, 70);  // prev 70
        AccelConstraint accStupidFast = new ProfileAccelConstraint(-100, 100);


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        //*****LOOP wait for start, INIT LOOP***********************************************************
        while (!isStarted() && !isStopRequested()) {

        }


        waitForStart();

        if (isStopRequested()) {
            sleep(1000);
        }

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());

    }
}