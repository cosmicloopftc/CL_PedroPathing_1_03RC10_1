package PID;



import com.arcrobotics.ftclib.controller.PIDController;
//This PID Controller is based on Kookybotz arm operation
//It uses the PIDController import from arcrobotics.ftclib
//NOT based on elapsed time.

public class PIDfeedforwardController {
    private PIDController controller;
    private final double feedForwardMotorPowerLocal;
    private final double ticks_in_degree;
    public double power;


    //public static double Kp = .045, Ki = 0, Kd = 0.0001;  //initial value to start in FtcDashboard; update these values here once we find the best values
    //public static double feedForwardPower = 0;            //public static variables will appear in FtcDashboard Configuration to allow adjustment
    //public static int targetPosition = 10;                //public static variables will appear in FtcDashboard Configuration to allow adjustment
    //private final double ticks_in_degree = 2786.2 / 360.0; //adjust this based on tick/360 degree spec for our motor (motor spec (encoder resolution) from Gobilda site)

    public PIDfeedforwardController(double Kp, double Ki, double Kd, double motorEncoderResolution,
                                        double feedForwardMotorPower) {
        ticks_in_degree = motorEncoderResolution / 360.0;
        controller.setPID(Kp, Ki, Kd);
        feedForwardMotorPowerLocal = feedForwardMotorPower;
    }

    public double setArmMotorPowerPIDF(int presentPosition, int targetPosition) {
        double pid = controller.calculate(presentPosition, targetPosition);
        double feedForward = Math.cos(Math.toRadians(ticks_in_degree)) * feedForwardMotorPowerLocal;
        power = pid + feedForward;
        return power;
    }
}