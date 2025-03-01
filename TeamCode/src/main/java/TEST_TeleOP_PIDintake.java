import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import PID.PIDsimpleController;
import com.arcrobotics.ftclib.controller.PIDController;


/** This case extend from BLUE alliance TeleOp program
 * 3/2/2025        TODO: Need to test
 *
 *
 * */

//@Config
//@TeleOp(group="test", name= "TEST_TeleOP_PIDintake 1.1")


public class TEST_TeleOP_PIDintake extends BLUE_TeleOpV1 {

    private PIDController intakePIDcontroller;
    public static double intakePID_Kp = 0.4, intakePID_Ki = 0, intakePID_Kd = 0;
    public static double feedforwardPower = 0;
    public static int intakeSliderTargetPosition = 0;
    private final double ticks_in_degree = 145.1/360;  //for out Intake motor


    @Override
    public void init() {
        super.init();

        //TODO: check if these variables should be here or in init()
        String allianceColor = "RED";           // To override set value, this need to be after super.init ?
        String nonAllianceColor = "BLUE";       // To override set value, this need to be after super.init ?

        intakePIDcontroller = new PIDController(intakePID_Kp,intakePID_Ki,intakePID_Kd);

    }

    @Override
    public void init_loop(){
        super.init_loop();

        intakePIDcontroller.setPID(intakePID_Kp,intakePID_Ki,intakePID_Kd);
        int intakeSliderPresentPosition = robot.Intake.intakeSlides.getCurrentPosition();
        double pid = intakePIDcontroller.calculate(intakeSliderPresentPosition, intakeSliderTargetPosition);
        double ff = Math.cos(Math.toRadians(intakeSliderTargetPosition/ticks_in_degree))*feedforwardPower;
        robot.Intake.intakeSlides.setPower(pid+ff);

        telemetryA.addData("intakeSliderPresentPosition = ", intakeSliderPresentPosition);
        telemetryA.addData("intakeSliderTargetPosition = ", intakeSliderTargetPosition);
        telemetryA.addData("intakeSlider Current(mA) = ", robot.Intake.getIntakeSlideCurrent());
    }




    public void start(){
        super.start();
    }

    public void loop(){
        super.loop();
    }

    public void stop(){
        super.stop();
    }

}