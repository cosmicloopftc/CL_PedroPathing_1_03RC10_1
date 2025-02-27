package Hardware;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareOuttake {
    //    private DcMotor Intake_Motor = null;
    public DcMotorEx outtakeLeftSlide = null;
    public DcMotorEx outtakeRightSlide = null;

    public Servo outtakeArmAxon = null;
    public Servo outtakeExtension = null;
    public Servo claw = null;
    public AnalogInput outtakeArmAxonPosition = null;
    /*Constructor*/
    public HardwareOuttake() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map
        //map and setup mode of slide motors
        outtakeLeftSlide = hardwareMap.get(DcMotorEx.class, "outtakeLeftSlide");
        outtakeLeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeLeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeLeftSlide.setDirection(DcMotorEx.Direction.FORWARD); //It is forward on robot
        outtakeLeftSlide.setPower(0);

        outtakeRightSlide = hardwareMap.get(DcMotorEx.class, "outtakeRightSlide");
        outtakeRightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeRightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeRightSlide.setDirection(DcMotorEx.Direction.REVERSE); //It is reversed on robot
        outtakeRightSlide.setPower(0);

        outtakeArmAxon = hardwareMap.get(Servo.class, "outtakeArmAxon");
        outtakeArmAxon.setDirection(Servo.Direction.REVERSE);
        outtakeExtension = hardwareMap.get(Servo.class, "outtakeExtension");
        outtakeExtension.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");


//get our analog input from the hardwareMap
        outtakeArmAxonPosition = hardwareMap.get(AnalogInput.class, "outtakeArmAxonPosition");
    }


    public void start(){

    }


    public void stop () {

    }
    public void leftSlideSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        outtakeLeftSlide.setTargetPosition(DesiredSliderPosition);
        outtakeLeftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeLeftSlide.setPower(SliderPower);
    }
    public void rightSlideSetPositionPower(
            int DesiredSliderPosition,
            double SliderPower){

        outtakeRightSlide.setTargetPosition(DesiredSliderPosition);
        outtakeRightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeRightSlide.setPower(SliderPower);
    }


    public void openClaw(){
        claw.setPosition(0.0);  // by Eduardo 2/17/25 (0.70); // all set for new bot
    }
    public void closeClaw(){
        claw.setPosition(0.30);  // by Eduardo 2/17/25 (1); // all set for new bot
    }
    public void extendIN(){
        outtakeExtension.setPosition(1);
    }
    public void extendOUT(){
        outtakeExtension.setPosition(0.85); //All the way out is 0.75, 0.85 is within the robot
    }
    public void groundPosition(){
        leftSlideSetPositionPower(0,1);
        rightSlideSetPositionPower(0,1);
        outtakeArmAxon.setPosition(0.34);  // by Eduardo 2/17/25 (0.34); //TODO: Find position for transfering (stays the same throughout process)
        extendIN();
        //openClaw();
    }
    public void groundPositionClose(){
        leftSlideSetPositionPower(0,0);
        rightSlideSetPositionPower(0,0);
        outtakeArmAxon.setPosition(0.3); //Figure out position for transfering (stays the same throughout process)
        extendIN();
        closeClaw();
    }

    public void readyPosition(){ // TODO: Do we need this?
        leftSlideSetPositionPower(0,1);
        rightSlideSetPositionPower(0,1);
        outtakeArmAxon.setPosition(0.43); //Figure out position for transfering
        extendIN();
        openClaw();
    }

    public void lowBasket(){
        //outtakeArmAxon.setPosition(0); //Should be same as high basket
        //outtakeExtension.setPosition(0); //No extension
        //leftSlideSetPositionPower(970,0.6);
        //rightSlideSetPositionPower(970,0.6);
    }
    public void highBasket(){
        outtakeArmAxon.setPosition(0.78);
        extendIN();
        leftSlideSetPositionPower(2375,1);
        rightSlideSetPositionPower(2375,1);
    }
    public void sampleDelivery(){
        outtakeArmAxon.setPosition(0.9);
        extendIN();
        leftSlideSetPositionPower(0,1);
        rightSlideSetPositionPower(0,1);
    }
    public void highChamberSetUpwards(){
        leftSlideSetPositionPower(1260,1);
        rightSlideSetPositionPower(1260,1);
        outtakeArmAxon.setPosition(0.9); //Should be same as wall intake
        extendIN();
    }
    public void highChamberFinishUpwards(){
        leftSlideSetPositionPower(1630,1); //Find position (Go down from high chamber set position)
        rightSlideSetPositionPower(1630,1);
        outtakeArmAxon.setPosition(0.9); //Should be same as wall intake
        extendIN();
    }
    public void wallIntakeFront(){
        leftSlideSetPositionPower(0,1);
        rightSlideSetPositionPower(0,1);
        outtakeArmAxon.setPosition(0.29);
        extendOUT();
    }

    public double getOuttakeSliderRightCurrent(){
        return outtakeRightSlide.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getOuttakeSliderLeftCurrent(){
        return outtakeLeftSlide.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getOuttakeArmPosition(){
        // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
        return outtakeArmAxonPosition.getVoltage() / 3.3;
    }

    public void slidersOnlyHighBasket(){
        leftSlideSetPositionPower(2400,1);
        rightSlideSetPositionPower(2400,1);
    }


}