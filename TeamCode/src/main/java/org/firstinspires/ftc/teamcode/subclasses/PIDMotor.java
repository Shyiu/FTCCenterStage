package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class PIDMotor extends Subsystem{
    MecanumBotConstant m = new MecanumBotConstant();
    DcMotorEx pid_motor;
    DigitalChannel magnet_sensor;
    public double targetPos;
    public static double P = 0.0005, I = 0.0002, D = 0;
    public static double F = 0;
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
    private double max_integral = 0;
    HardwareMap hardware;
    Telemetry telemetry;

    public PIDMotor(HardwareMap hardwareMap, Telemetry telemetry, String name) {
        this.hardware = hardwareMap;
        this.telemetry = telemetry;
        this.name = name;
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);

    }
    public void setPID(double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = 0;
    }
    public void setPIDF(double P, double I, double D, double F){
        setPID(P,I,D);
        this.F = F;
    }
    public void setMaxIntegral(double max_integral){
        this.max_integral = max_integral;
    }
    public void setDirection(DcMotor.Direction power){
        pid_motor.setDirection(power);
    }

    public void setMin(int min){
        minHardstop = min;
    }

    public void setMax(int max){
        maxHardstop = max;
    }



    @Override
    public void init(){
        setPower(0);
        pid_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void telemetry(){
        telemetry.addData(name + " Power", getPower());
        telemetry.addData(name + "'s Position", getCurrentPosition());
        telemetry.addData(name + "'s Target Position", targetPos);
        telemetry.addData(name + "'s Max Hardstop", maxHardstop);
        telemetry.addData(name + "'s Min Hardstop", minHardstop);
        telemetry.addData(name + "'s direction", direction);
        telemetry.addData(name + " Exceeding Constraints", exceedingConstraints());

    }


    public boolean exceedingConstraints(){
            boolean over_max = getCurrentPosition() > maxHardstop;
            boolean under_min = getCurrentPosition() < minHardstop;
            return getPower() > 0 ? over_max : under_min;

    }
    public boolean exceedingConstraints(double power){
        boolean over_max = getCurrentPosition() > maxHardstop;
        boolean under_min = getCurrentPosition() < minHardstop;
        return power > 0 ? over_max : under_min;

    }

    public void setPower(double power) {
        if(!exceedingConstraints(power)) {
            pid_motor.setPower(power + holding_power);
        }else{
            pid_motor.setPower(holding_power);
        }
    }

    public int getCurrentPosition() {
        return pid_motor.getCurrentPosition();
    }

    public void move_sync(double target, double timeoutS, double MOTOR_POWER) {
        double currentTime = System.currentTimeMillis();
        double slidesPosition = getCurrentPosition();
        if (slidesPosition > target) {
            setPower(-MOTOR_POWER);
            while (slidesPosition > target && System.currentTimeMillis() - currentTime < timeoutS * 1000 && !exceedingConstraints()) {
                slidesPosition = getCurrentPosition();
            }
            setPower(0);
        } else if (slidesPosition < target) {
            setPower(MOTOR_POWER);
            while (slidesPosition < target && System.currentTimeMillis() - currentTime < timeoutS * 1000  && !exceedingConstraints()) {
                slidesPosition = getCurrentPosition();
            }
            setPower(0);
        }
    }

    public void move_async(double target) {
        targetPos = target;
    }


    public double getPower(){
        return pid_motor.getPower();
    }

    public void update() {
        double Kp = P;
        double Ki = I;
        double Kd = D;

        double reference = targetPos;

        double integralSum = 0;
        ElapsedTime timer = new ElapsedTime();

        // obtain the encoder position
        double currentPosition = getCurrentPosition();
        // calculate the error
        error = reference - currentPosition;

        // rate of change of the error

        // sum of all error over time
        double out = (Kp * error);
        integralSum = integralSum + (error * timer.seconds());
        if (integralSum <= max_integral){
            out += (Ki * integralSum) ;
        }



        if(timer.seconds() != 0) {
            double derivative = (error - lastError) / timer.seconds();
            out += Kd * derivative;
        }
        setPower(out + Math.copySign(F, out));
        lastError = error;
        // reset the timer for next time
        timer.reset();


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
