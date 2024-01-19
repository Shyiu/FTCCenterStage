package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class PIDMotor extends Subsystem{
    MecanumBotConstant m = new MecanumBotConstant();
    DcMotorEx slides;
    DigitalChannel magnet_sensor;
    public double targetPos;
    public static double P = 0.0005, I = 0.0002, D = 0;
    double error, lastError;
    int startPos = Integer.MAX_VALUE;
    boolean direction = true;
    double GAIN = 30;
    public boolean stopped = false;
    private double powerReduction = 1;
    private double conversion = -1;
    private boolean reached = false;
    private boolean working_magnet;
    private ElapsedTime timer;
    private int maxHardstop = 1450;
    private String name = "";
    private int minHardstop = 0;
    private double holding_power = 0;
    HardwareMap hardware;
    Telemetry telemetry;

    public PIDMotor(HardwareMap hardwareMap, Telemetry telemetry, String name) {
        this.hardware = hardwareMap;
        this.telemetry = telemetry;
        this.name = name;
        slides = hardwareMap.get(DcMotorEx.class, name);

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotor.Direction.FORWARD);

    }
    public void setDirection(DcMotor.ZeroPowerBehavior power){
        slides.setZeroPowerBehavior(power);

    }

    public void setMin(int min){
        minHardstop = min;
    }
    public void setMax(int max){
        maxHardstop = max;
    }
    public void setHoldingPower(double power){
        holding_power = power;
    }
    @Override
    public void init(){
        slides.setPower(holding_power);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void telemetry(){
        telemetry.addData(name + " Power", getPower());
        telemetry.addData(name + "'s Position", getCurrentPosition());
    }


    public boolean exceedingConstraints(){

            return slides.getPower() > 0 ? slides.getCurrentPosition() > maxHardstop : slides.getCurrentPosition() < minHardstop ;

    }
    public boolean exceedingConstraints(double power){

        return power > 0? slides.getCurrentPosition() > maxHardstop : slides.getCurrentPosition() < minHardstop ;

    }

    public void setPower(double power) {
        if(!exceedingConstraints(power)) {
            slides.setPower(power);
        }else{
            slides.setPower(holding_power);
        }
    }

    public int getCurrentPosition() {
        return slides.getCurrentPosition();
    }

    public void control(double target, double timeoutS, double SLIDE_POWER) {
        double currentTime = System.currentTimeMillis();
        double slidesPosition = slides.getCurrentPosition();
        if (slidesPosition > target) {
            slides.setPower(-SLIDE_POWER);
            while (slidesPosition > target && System.currentTimeMillis() - currentTime < timeoutS * 1000 && !exceedingConstraints()) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(holding_power);
        } else if (slidesPosition < target) {
            slides.setPower(SLIDE_POWER);
            while (slidesPosition < target && System.currentTimeMillis() - currentTime < timeoutS * 1000  && !exceedingConstraints()) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(holding_power);
        }
    }
    public void bringToHalt(){
        double power = 0.4;
        setPower(0.4);
        double velocity = slides.getVelocity();
        while (velocity > 1){
            ;
        }
        setPower(0);
        init();
    }
    public void moveTo(double target) {
        reached = false;
        targetPos = target;
        direction = getCurrentPosition() < target;
    }
    public double getPower(){
        return slides.getPower();
    }


    public boolean isBusy(){
        return direction ? getCurrentPosition() < targetPos : getCurrentPosition() > targetPos;

    }
    public boolean inRange(int target, int position, int range){
        int min = target - range;
        int max = target + range;
        return min<=position && position<=max;
    }
    public void update() {
        double Kp = P;
        double Ki = I;
        double Kd = D;

        double reference = targetPos;

        double integralSum = 0;
        if (isBusy() && !reached && !exceedingConstraints()) {
// Elapsed timer class from SDK, please use it, it's epic
            ElapsedTime timer = new ElapsedTime();

            // obtain the encoder position
            double encoderPosition = slides.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            slides.setPower(out/powerReduction);

            // reset the timer for next time
            timer.reset();
        }
        else {
            slides.setPower(holding_power);
            reached = true;
        }
    }


//    public double getBatteryVoltage() {
//        double result = Double.POSITIVE_INFINITY;
//        for (VoltageSensor sensor : hardware.voltageSensor) {
//            double voltage = sensor.getVoltage();
//            if (voltage > 0) {
//                result = Math.min(result, voltage);
//            }
//        }
//        return result;
//    }
}
