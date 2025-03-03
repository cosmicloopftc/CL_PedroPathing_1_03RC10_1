import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import Hardware.HardwareDrivetrain;
import Hardware.HardwareNoDriveTrainRobot;

public class Diagnostic extends OpMode {

    //Define
    private Servo[] axonServos;
    private AnalogInput[] axonAnalogInput;
    private DcMotorEx[] drivetrainMotors;

    //Define names on configuration
    private final String[] ServoNames = {"intakeServoAxon", "outtakeArmAxon"};
    private final String[] AxonAnalogInputNames = {"intakeArmAxonPosition", "outtakeArmAxonPosition"};
    private final String[] DriveTrainMotorNames = {"leftFront", "rightFront", "leftBack", "rightBack"};

    private final ElapsedTime timer = new ElapsedTime();
    private int state = 0;
    private int currentServo = 0;

    HardwareNoDriveTrainRobot robot = new HardwareNoDriveTrainRobot();
    HardwareDrivetrain drivetrain = new HardwareDrivetrain();

    @Override
    public void init() {
        resetRobot();
        //Initializes names of devices and assigns them to a device
        axonServos = new Servo[ServoNames.length];
        for (int i = 0; i < ServoNames.length; i++) {
            axonServos[i] = hardwareMap.get(Servo.class, ServoNames[i]);

            //Comment out if directions are normal
            axonServos[i].setDirection(Servo.Direction.REVERSE);
        }

        axonAnalogInput = new AnalogInput[AxonAnalogInputNames.length];
        for (int i = 0; i < AxonAnalogInputNames.length; i++) {
            axonAnalogInput[i] = hardwareMap.get(AnalogInput.class, AxonAnalogInputNames[i]);
        }

        drivetrainMotors = new DcMotorEx[DriveTrainMotorNames.length];
        for (int i = 0; i < DriveTrainMotorNames.length; i++) {
                drivetrainMotors[i] = hardwareMap.get(DcMotorEx.class, DriveTrainMotorNames[i]);
                drivetrainMotors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                drivetrainMotors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                drivetrainMotors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double elapsedTime = timer.seconds();
        double DELAY_TIME = 1.0;

        switch (state) {
            case 0:
                telemetry.addData("Press Button for Diagnostic ", "Right Bumper for Help");
                telemetry.update();
                timer.reset();
                if (gamepad1.right_bumper) {
                    state = 11;
                } else if (gamepad1.x) {
                    currentServo = 0;
                    state = 1;
                } else if (gamepad1.y) {
                    currentServo = 1;
                    state = 5;
                } else if (gamepad1.a) {
                    state = 13;
                }
                break;

            /** Outtake Axon*/
            case 1:
                telemetry.addData("Status:", "Starting diagnostic check for Outtake Servo");
                telemetry.update();
                state++;
                break;

            case 2:
                axonServos[currentServo].setPosition(0.34);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("Low Position", "0.34");
                    telemetry.addData("Measured Position", getAxonPosition(axonAnalogInput[currentServo]));
                    timer.reset();
                    state++;
                }
                break;

            case 3:
                axonServos[currentServo].setPosition(0.6);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("Middle Position", "0.6");
                    telemetry.addData("Measured Position", getAxonPosition(axonAnalogInput[currentServo]));
                    timer.reset();
                    state++;
                }
                break;

            case 4:
                axonServos[currentServo].setPosition(0.9);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("High Position", "0.9");
                    telemetry.addData("Measured Position", getAxonPosition(axonAnalogInput[currentServo]));
                    timer.reset();
                    state = 12;
                }
                break;

                /** Intake Axon*/
            case 5:
                telemetry.addData("Status:", "Starting diagnostic check for Intake Servo");
                telemetry.update();
                state++;
                break;

            case 6:
                axonServos[currentServo].setPosition(0.4);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("Inside position", "0.4");
                    telemetry.addData("Measured Position", getAxonPosition(axonAnalogInput[currentServo]));
                    timer.reset();
                    state++;
                }
                break;

            case 7:
                axonServos[currentServo].setPosition(0.72);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("Middle", "0.72");
                    telemetry.addData("Measured Position", getAxonPosition(axonAnalogInput[currentServo]));
                    timer.reset();
                    state++;
                }
                break;

            case 8:
                axonServos[currentServo].setPosition(0.97);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("Low position", "0.97");
                    telemetry.addData("Measured Position", getAxonPosition(axonAnalogInput[currentServo]));
                    timer.reset();
                    state = 12;
                }
                break;

                /**Drivetrain*/
            case 13:
                telemetry.addData("Status:", "Starting diagnostic check for drivetrain");
                telemetry.update();
                state++;
                break;

            case 14:
                HardwareDrivetrain.setMotorPower(0,0.3,0,0);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("LeftFrontMotor", "30% Power");
                    timer.reset();
                    state++;
                }
                break;

            case 15:
                HardwareDrivetrain.setMotorPower(0.3,0,0,0);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("RightFrontMotor", "30% Power");
                    timer.reset();
                    state++;
                }
                break;

            case 16:
                HardwareDrivetrain.setMotorPower(0,0,0,-0.3);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("LeftBackMotor", "30% Power (Negative)");
                    timer.reset();
                    state++;
                }
                break;

            case 17:
                HardwareDrivetrain.setMotorPower(0,0,-0.3,0);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("RightBackMotor", "30% Power (Negative)");
                    timer.reset();
                    state++;
                }
                break;
            case 18:
                HardwareDrivetrain.setMotorPower(0,0,0,0);
                if (elapsedTime >= DELAY_TIME) {
                    telemetry.addData("Motor Test Complete", "Motor Stopped");
                    timer.reset();
                    state = 12;
                }
                break;


                /**Final*/
            case 11:
                telemetry.addData("Help:", "X: Intake Servo | Y: Outtake Servo | A: Drivetrain Test");
                telemetry.update();
                state = 0;
                break;

            case 12:
                if (elapsedTime >= DELAY_TIME) {
                    resetRobot();
                    telemetry.addData("Status:", "Diagnostic Complete.");
                    telemetry.update();
                    state = 0;
                }
                break;
        }
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "OpMode Stopped.");
        telemetry.update();
    }

    public void resetRobot() {
        robot.Outtake.extendIN();
        robot.Outtake.outtakeArmAxon.setPosition(0.43);
        robot.Outtake.closeClaw();
        robot.Intake.intakeINSIDEBOT();
    }
    public double getAxonPosition(AnalogInput servo){
        return servo.getVoltage() / 3.3;
    }
}
