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
    private boolean releasing_hooks = false;

    private boolean activated = false;

    private boolean touchSensorLimit = true;

    private boolean disable_update = false;

    private boolean start_move = false;
    public static double P = 0.006;
    private double expected_run_time = .3;
    private double hook_time = 0;
    private double release_time = 0;
    ElapsedTime timer;
    private enum HOOK_STATE{
        EXTEND, RETRACT, COMPLETE
    }
    private HOOK_STATE release_state = HOOK_STATE.EXTEND;
    public static double HOOK_POWER = 1;
    private String state = "Init";
    Telemetry telemetry;

    public ShivaniRigging(HardwareMap hardwareMap, Telemetry telemetry){
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        riggingMotor = new PIDMotor(hardwareMap, telemetry, mc.rigging_motor);
        hookMotor = new PIDMotor(hardwareMap, telemetry,  mc.hook_motor);
        touchSensor = hardwareMap.get(TouchSensor.class, mc.limit_switch);


        riggingMotor.setDirection(DcMotor.Direction.FORWARD);
        riggingMotor.setMin(-2200);
        riggingMotor.setReversedEncoder(true);
        riggingMotor.setMaxIntegral(1);

        hookMotor.setDirection(DcMotor.Direction.REVERSE);
        hookMotor.pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hookMotor.setMax(280);
        hookMotor.setMin(-50);
        hookMotor.P = P;
        hookMotor.I = 0;



        magnet_sensor = hardwareMap.get(DigitalChannel.class, mc.magnet_sensor);
        // set the digital channel to input.
        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);






    }
    public void setHookPower(double power){
        if(power > 0){
            if(magnet_activated()){
                hookMotor.setAbsPower(0);
            }else {
                hookMotor.setAbsPower(power);
            }
        }else{
            hookMotor.setAbsPower(power);
        }
    }
    public void openHooks(){
        setHookPower(HOOK_POWER);
    }
    public void closeHooks(){
        setHookPower(-0.3);
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
        riggingMotor.telemetry();
        hookMotor.telemetry();
        telemetry.addData("Magnet Activated", magnet_activated());
        telemetry.addData("Hook Speed", hookMotor.getPower());
        telemetry.addData("State: ", state);

    }
    public boolean isBusy(){
        return activated && hookMotor.isBusy();
    }
    public void raise_hooks_to_sensor(){
        hook_time = timer.time();
        activated = true;
    }


    public void release_motor(){

        setHookPower(0);
        hookMotor.pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void update(){

        if(activated){
            if(magnet_activated()){
                setHookPower(0);
                hookMotor.init();
                setHookPower(-.3);
                try {
                    sleep(200);
                }catch(InterruptedException e){

                }
                disable_update = true;

                activated = false;
                //                release_motor();
            }

            double elapsed_time = timer.time() - hook_time;
            double motor_power = Math.max(0.35, HOOK_POWER * (expected_run_time - elapsed_time)/expected_run_time);
            setHookPower(motor_power);

        }else{
            if(!disable_update) {
                hookMotor.update();
            }
        }
        if(releasing_hooks){
            switch(release_state){
                case EXTEND:
                    if(!hookMotor.isBusy()){
                        hookMotor.move_async(0);
                        release_time = timer.time();
                        release_state = HOOK_STATE.RETRACT;
                    }
                    break;
                case RETRACT:
                    if(!hookMotor.isBusy()){
                        setHookPower(-.4);
                        release_state = HOOK_STATE.COMPLETE;
                        timer.reset();

                    }
                    break;
                case COMPLETE:
                    if(timer.time() > 0.3){
                        setHookPower(0);
                        releasing_hooks = false;
                        break;

                    }

            }
        }
    }
    public int getHookPosition(){
        return hookMotor.getCurrentPosition();
    }
    public void moveHook(int target){
        hookMotor.move_async(target);
    }
    public void release_hooks(){
        hookMotor.init();
        hookMotor.move_async(150);
        releasing_hooks = true;
        release_state = HOOK_STATE.EXTEND;

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
        riggingMotor.setMax(0);
        hookMotor.init();


        resetEncoder();

    }
}
