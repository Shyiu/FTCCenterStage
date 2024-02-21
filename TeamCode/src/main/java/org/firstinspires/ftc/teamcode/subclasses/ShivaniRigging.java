package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class ShivaniRigging extends Subsystem{
    PIDMotor riggingMotor;
    DcMotorSimple left_servo, right_servo;
    TouchSensor touchSensor;  // Touch sensor Object
    DigitalChannel magnet_sensor;

    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;
    private boolean up = false;
    private boolean touchSensorLimit = true;
    double error, lastError;

    public static double P = 20, I = 3, D = 0;
    public static double target_position = -2000;
    ElapsedTime timer;
    private double delay = .5;

    public static double hold_speed = 0;
    private String state = "Init";
    Telemetry telemetry;
    public static double LEFT_OPEN_POWER = .6, RIGHT_OPEN_POWER = -0.1;
    public static double LEFT_CLOSE_POWER = 0, RIGHT_CLOSE_POWER = 0;

    //-1 to rig left, 1 to rig right.

    public ShivaniRigging(HardwareMap hardwareMap, Telemetry telemetry){
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        riggingMotor = new PIDMotor(hardwareMap, telemetry, mc.rigging_motor);
        touchSensor = hardwareMap.get(TouchSensor.class, mc.limit_switch);

        riggingMotor.setDirection(DcMotor.Direction.FORWARD);
        riggingMotor.setMin(-2200);
        riggingMotor.setReversedEncoder(true);
        riggingMotor.setMaxIntegral(1);

//        magnet_sensor = hardwareMap.get(DigitalChannel.class, mc.magnet_sensor);
//        // set the digital channel to input.
//        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);





        left_servo = hardwareMap.get(DcMotorSimple.class, mc.rigging_left);
        right_servo = hardwareMap.get(DcMotorSimple.class, mc.rigging_right);
    }
    public void set_left_power(double power){
        left_servo.setPower(power);
    }

    private boolean magnet_activated(){
        return !magnet_sensor.getState();
    }

    public void set_right_power(double power){
        right_servo.setPower(power);
    }

    public void setRiggingPower(double speed){
        if (touchSensorLimit && touchSensor.isPressed() && speed < 0){
            riggingMotor.setPower(0);
        }else {
            riggingMotor.setPower(speed);
        }
    }


    @Override
    public void telemetry() {
        riggingMotor.telemetry();
        telemetry.addData("State: ", state);

    }
    public boolean isBusy(){
        return up ? riggingMotor.getCurrentPosition() < target_position : riggingMotor.getCurrentPosition() > target_position;
    }
    public void activate(){
        riggingMotor.move_async(target_position);
    }

    public void update(){
        if(magnet_activated()){
            set_left_power(0);
            set_right_power(0);
        }
    }
    public void openBoth(){
        set_right_power(RIGHT_OPEN_POWER);
        set_left_power(LEFT_OPEN_POWER);
    }
    public void stop(){
        set_right_power(0);
        set_left_power(0);
    }
    public void addSlack(){
        set_right_power(RIGHT_CLOSE_POWER);
        set_left_power(LEFT_CLOSE_POWER);
    }

    public void resetEncoder() {
        timer = new ElapsedTime();

        timer.reset();
        if(touchSensor.isPressed()){
            riggingMotor.init();
            return;
        }
        while (!touchSensor.isPressed()) {
            if(timer.time() > 4){
                telemetry.addLine("Suspecting Disconnected Touch Sensor. Aborting init.");
                touchSensorLimit = false;
                break;
            }
            riggingMotor.setAbsPower(-0.4);
        }
        riggingMotor.setPower(0);
        riggingMotor.init();

    }
    public void raise_hooks() throws InterruptedException {
        set_left_power(LEFT_OPEN_POWER);
        set_right_power(RIGHT_OPEN_POWER);
        sleep(500);
        set_left_power(0);
        set_right_power(0);
    }
    @Override
    public void init() {
        riggingMotor.P = P;
        riggingMotor.I = I;
        riggingMotor.D = D;
        riggingMotor.setMax(0);
        set_left_power(0);
        set_right_power(0);
        resetEncoder();

    }
}
