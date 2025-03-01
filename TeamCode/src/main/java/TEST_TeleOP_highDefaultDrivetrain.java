import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/** This case extend from BLUE alliance TeleOp program
 * 3/3/2025        TODO: Need to test
 *
 *
 * */

@Config
@TeleOp(group="test", name= "TEST_highDefaultDrivetrain 1.1")


public class TEST_TeleOP_highDefaultDrivetrain extends BLUE_TeleOpV1 {


    @Override
    public void init() {
        super.init();
        motorPowerDefault = 0.5;

    }

    @Override
    public void init_loop(){
        super.init_loop();
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
