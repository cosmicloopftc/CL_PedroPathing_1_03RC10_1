package Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure

public class HardwareHang {
    public DcMotor hangMotor = null;

    /*Constructor*/
    public HardwareHang() {
    }

    /* Initialize standard Hardware interface */
    public void init(HardwareMap hardwareMap)    {
        //Save reference to Hardware map

//example:  map and setup mode of Intake motor
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse if 0 is at ready to hang position, forward if 0 inside robot
        hangMotor.setPower(0);


    }


    public void start(){

    }


    public void stop () {

    }

//example:  for Intake motor
//public method (function) for Intake motor power--to be accessible from anywhere
    public void setPowerPosition(int position, double power) {
        hangMotor.setTargetPosition(position);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotor.setPower(power);
    }
    public void hang() {
        setPowerPosition(2500, 1); //Make sure 0 is when hooks all the way extended back
    }
}