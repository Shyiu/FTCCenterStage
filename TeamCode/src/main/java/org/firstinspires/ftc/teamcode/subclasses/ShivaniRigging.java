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
    PIDMotor hookMotor;
    TouchSensor touchSensor;  // Touch sensor Object
    DigitalChannel magnet_sensor;

    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;
    private boolean up = false;
    private boolean activated = false;
    private boolean touchSensorLimit = true;

    public static double P = 0.0025, I = 1/(210.0 * 2), D = 0;
    public static double target_position = 162;
    ElapsedTime timer;
    private double delay = .5;

    public static double hold_speed = 0;
    private String state = "Init";
    Telemetry telemetry;

    public ShivaniRigging(HardwareMap hardwareMap, Telemetry telemetry){
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        riggingMotor = new PIDMotor(hardwareMap, telemetry, mc.rigging_motor);
        hookMotor = new PIDMotor(hardwareMap, telemetry, mc.hook_motor);
        touchSensor = hardwareMap.get(TouchSensor.class, mc.limit_switch);


        riggingMotor.setDirection(DcMotor.Direction.FORWARD);
        riggingMotor.setMin(-2200);
        riggingMotor.setReversedEncoder(true);
        riggingMotor.setMaxIntegral(1);

        hookMotor.setDirection(DcMotor.Direction.REVERSE);
        hookMotor.setMin(-60);
        hookMotor.setMax(270);
        hookMotor.pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        magnet_sensor = hardwareMap.get(DigitalChannel.class, mc.magnet_sensor);
//        // set the digital channel to input.
//        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);





    }
    public void setHookPower(double power){
        hookMotor.setPower(power);
    }

    private boolean magnet_activated(){
        return !magnet_sensor.getState();
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
        hookMotor.telemetry();
        riggingMotor.telemetry();
        telemetry.addData("State: ", state);

    }
    public boolean isBusy(){
        return activated && hookMotor.isBusy();
    }
    public boolean isCompletedFor(double time){
        return activated && hookMotor.isCompletedFor(time);
    }
    public void activate(){
        hookMotor.move_async(target_position);
        activated = true;
    }

    public void release_motor(){
        hookMotor.pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void update(){
        hookMotor.update();
    }
    public void set_hook_target_position(double target_position){
        hookMotor.move_async(target_position);
        activated= true;
    }
    public void release_hooks(){
        hookMotor.move_sync(120,3,0.6);
        hookMotor.move_sync(-60 ,3,0.2);
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

    @Override
    public void init() {
        hookMotor.I = I;
        hookMotor.P = P;
        hookMotor.D = D;
        riggingMotor.setMax(0);
        hookMotor.init();


        resetEncoder();

    }
}
