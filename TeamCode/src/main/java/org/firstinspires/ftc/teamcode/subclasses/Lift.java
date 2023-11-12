package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Lift {
    MecanumBotConstant m = new MecanumBotConstant();
    DcMotor slides;
    DigitalChannel magnet_sensor;
    public double targetPos;
    public static double P = 0, I = 0, D = 0;
    double error, lastError;
    int startPos = Integer.MAX_VALUE;
    boolean up = true;
    double GAIN = 30;
    public boolean stopped = false;
    private double powerReduction = 1;
    private double conversion = -1;
    private boolean reached = false;
    private int maxHardstop = 2000;

    HardwareMap hardware;

    public Lift(HardwareMap hardwareMap, double P, double I, double D) {

        this.hardware = hardwareMap;
        this.P = P;
        this.I = I;
        this.D = D;

        slides = hardwareMap.get(DcMotor.class, m.slides);
        magnet_sensor = hardwareMap.get(DigitalChannel.class, m.magnet);

        // set the digital channel to input.
        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotor.Direction.FORWARD);

    }

    public void reset(){
        if(!magnet_sensor.getState()){
            slides.setPower(-.3);
        }
        while(magnet_sensor.getState()){
            slides.setPower(-.3);
        }
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean getMagnet(){
        return magnet_sensor.getState();
    }
    public boolean exceedingConstraints(){
        if(slides.getPower() < 0){
            return !magnet_sensor.getState();
        }else{
            return slides.getCurrentPosition() > maxHardstop;
        }
    }

    public void setPower(double power) {
        if(!exceedingConstraints()) {
            slides.setPower(power);
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
            slides.setPower(0);
        } else if (slidesPosition < target) {
            slides.setPower(SLIDE_POWER);
            while (slidesPosition < target && System.currentTimeMillis() - currentTime < timeoutS * 1000  && !exceedingConstraints()) {
                slidesPosition = slides.getCurrentPosition();
            }
            slides.setPower(0);
        }
    }

    public void moveTo(double target) {
        reached = false;
        targetPos = target;
    }
    public double getPower(){
        return slides.getPower();
    }
    public boolean getReached(){
        return reached;
    }
    public boolean isBusy(){
        return Math.abs(getCurrentPosition()) < Math.abs(targetPos);
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
        else if(!reached || exceedingConstraints()){
            slides.setPower(0);
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
