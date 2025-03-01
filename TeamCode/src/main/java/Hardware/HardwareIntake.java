package Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//TODO: program the Servo position


public class HardwareIntake {

    public DcMotorEx intakeSlides = null;

    public CRServo intakeLeftWheel = null;
    public CRServo intakeRightWheel = null;
    public Servo intakeServoAxon = null;
    public Servo sweeper = null;
    public AnalogInput intakeServoAxonPosition = null;
    //Servo Test
//    public Servo Servo_Test = null;


    /*Constructor*/
    public HardwareIntake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map

        //map and setup mode of Intake Slide Motor
        intakeSlides = hardwareMap.get(DcMotorEx.class, "intakeSlides");
        //intake slide motor behaviors
        intakeSlides.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeSlides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        Slider_Motor_Right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        Slider_Motor_Left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlides.setDirection(DcMotorEx.Direction.REVERSE);
        intakeSlides.setPower(0);

        //map and setup mode of Intake Servos
        intakeLeftWheel = hardwareMap.get(CRServo.class, "intakeLeftWheel");
        intakeRightWheel = hardwareMap.get(CRServo.class, "intakeRightWheel");
        intakeServoAxon = hardwareMap.get(Servo.class,"intakeServoAxon");

        sweeper = hardwareMap.get(Servo.class, "sweeper");

//get our analog input from the hardwareMap
        intakeServoAxonPosition = hardwareMap.get(AnalogInput.class, "intakeServoAxonPosition");

        //TEST SERVO
//        Servo_Test = hardwareMap.get(Servo.class, "Servo_Test");

    }


    public void start(){

    }


    public void stop () {

    }

    //TODO: Method for entire intake process
    //public method (function) for intaking sample
    public void intakeSlideSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        intakeSlides.setTargetPosition(DesiredSliderPosition);
        intakeSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlides.setPower(SliderPower);
    }

    //public method (function) for intaking in sample
    public void intakeIN() {
        intakeLeftWheel.setPower(-1);
        intakeRightWheel.setPower(1);
    }


    //public method (function) for spitting out sample
    public void intakeOUT() {
        intakeLeftWheel.setPower(1);
        intakeRightWheel.setPower(-1);
    }

    //public method (function) for stopping the intake
    public void intakeSTOP() {
        intakeLeftWheel.setPower(0);
        intakeRightWheel.setPower(0);
    }

    //method for the retracted position of the intake slides
    public void intakeSlideIN() {
        intakeSlideSetPositionPower(0,0.4); //Inside robot
    }

    //method for the extended position of the intake slides
    public void intakeSlideOUT() {
        intakeSlideSetPositionPower(350,0.8); //Far out position (within extension limit)
    }
    public void intakeSlideMID() {
        intakeSlideSetPositionPower(230,0.8); //Middle position
    }

    public void intakeDOWN(){
        intakeServoAxon.setPosition(1); //Position when intaking
    }
    public void intakeUP(){
        intakeServoAxon.setPosition(0.87); //Position when down, but not intaking
    }
    public void intakeINSIDEBOT(){
        //intakeServoAxon.setPosition(0.52); //Position when grabbing specimen off wall
    }
    public void intakeTRANSFER(){
        intakeServoAxon.setPosition(0.635); //Position where sample gets grabbed out of intake
    }
    public double getIntakeServoAxonPosition(){
        // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
        return intakeServoAxonPosition.getVoltage() / 3.3;
    }
    public void sweeperIN(){
        sweeper.setPosition(0.09);
    }
    public void sweeperOUT(){
        sweeper.setPosition(0.5);
    }

    public double getIntakeSlideCurrent(){
        return intakeSlides.getCurrent(CurrentUnit.MILLIAMPS);
    }
}