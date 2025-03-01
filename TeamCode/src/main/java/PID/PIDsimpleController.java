package PID;

//modified from FTC Thunderbolts (Sacramento, CA) mentor's program structure
//**PID Controller example starts at 1:01:00 time mark, JAVA code starts at 1:16:00 to 1:33:00

public class PIDsimpleController {
    double Kp;
    double Ki;
    double Kd;
    double targetValue;
    double error = 0;
    double errorSum = 0;
    double errorDiff = 0;
    double errorPrevious = 0;

    /*Constructor*/
    public PIDsimpleController(double targetVal, double Kp_in, double Ki_in, double Kd_in) {
        Kp = Kp_in;
        Ki = Ki_in;
        Kd = Kd_in;
        targetValue = targetVal;
    }
    public double update(double input) {
        double output;
        error = input - targetValue;
        errorSum +=error;
        errorDiff = error - errorPrevious;
        output = Kp*error +Ki*errorSum + Kd*errorDiff;
        errorPrevious = error;
        return output;
    }
}