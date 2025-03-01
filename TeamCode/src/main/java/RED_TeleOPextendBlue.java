import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** This case extend from BLUE alliance TeleOp program
 * 2/23/2025        TODO: Need to test
 *
 *
 * */

@Config
@TeleOp(group="Primary", name= "RED TeleOp by extendclass 1.1")


public class RED_TeleOPextendBlue extends BLUE_TeleOpV1 {


    @Override
    public void init() {
        super.init();

        //TODO: check if these variables should be here or in init()
        String allianceColor = "RED";           // To override set value, this need to be after super.init ?
        String nonAllianceColor = "BLUE";       // To override set value, this need to be after super.init ?

    }

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
