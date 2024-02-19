package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Delivery extends Subsystem{
    protected Servo delivery, plunger;
    protected MecanumBotConstant m;
    private ElapsedTime timer;

    private double servoTimer1 = 0;
    private double servoTimer2 = 0;

    private boolean left_toggle = false;
    private boolean right_toggle = false;

    public double servoDelay = .5;
    public double rightTimer = 0;
    public double leftTimer = 0;


    public static double LEFT_INTAKE = 0;
    public static double PLUNGER_IN = 0;

    private static double plunger_position = 0;
    private static double delivery_position = 0;

    Telemetry telemetry;
    public Delivery(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;


        delivery = hardwareMap.get(Servo.class, m.delivery);
        plunger = hardwareMap.get(Servo.class, m.clutch);
        timer = new ElapsedTime();
    }

    public void openRight(){
        right_toggle = true;
        rightTimer = timer.time();
    }
    public void openLeft(){
        left_toggle = true;
        leftTimer = timer.time();
    }
    public boolean far(double target, double position){
        double max = position + .05;
        double min = position - .05;
        return target > max || target < min;

    }
    public void update(){
        if(timer.time() - leftTimer > servoDelay){
            left_toggle = false;
        }
        if(timer.time() - rightTimer > servoDelay){
            right_toggle = false;
        }

    }


    @Override
    public void telemetry() {
        telemetry.addData("Delivery Position: ", delivery_position);
        telemetry.addData("Plunger Position: ", plunger_position);
    }
    public void goToPosition(double position){
        delivery_position = position;
        delivery.setPosition(position);
    }

    @Override
    public void init() {
        delivery_position = LEFT_INTAKE;
        delivery.setPosition(LEFT_INTAKE);
        plunger_position = PLUNGER_IN;
        plunger.setPosition(PLUNGER_IN);

    }
}
