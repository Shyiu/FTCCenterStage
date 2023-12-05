package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Delivery extends Subsystem{
    protected Servo dleft;
    protected Servo dright;
    protected MecanumBotConstant m;
    private ElapsedTime timer;

    private double servoTimer1 = 0;
    private double servoTimer2 = 0;

    private boolean left_toggle = false;
    private boolean right_toggle = false;

    public double servoDelay = .5;
    public double rightTimer = 0;
    public double leftTimer = 0;


    public static double RIGHT_DUMP = 1;
    public static double LEFT_DUMP = 1;
    public static double RIGHT_INTAKE = 0;
    public static double LEFT_INTAKE = 0;

    Telemetry telemetry;
    public Delivery(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;
        RIGHT_DUMP = m.right_open_position;
        LEFT_DUMP = m.left_open_position;
        RIGHT_INTAKE = m.right_closed_position;
        LEFT_INTAKE = m.left_closed_position;

        dleft = hardwareMap.get(Servo.class, m.delivery_left);
        dright = hardwareMap.get(Servo.class, m.delivery_right);
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
        if (left_toggle){
            if(far(dleft.getPosition(), LEFT_INTAKE)) {
                dleft.setPosition(LEFT_INTAKE);
            }
        }else{
            if(far(dleft.getPosition(), LEFT_DUMP)) {
                dleft.setPosition(LEFT_DUMP);
            }
        }if (right_toggle){
            if(far(dright.getPosition(), RIGHT_INTAKE)) {
                dright.setPosition(RIGHT_INTAKE);
            }
        }else{
            if(far(dright.getPosition(), RIGHT_DUMP)) {
                dright.setPosition(RIGHT_DUMP);
            }
        }
    }


    @Override
    public void telemetry() {
        telemetry.addData("Right Open", right_toggle);
        telemetry.addData("Left Open", left_toggle);
    }

    @Override
    public void init() {
        dleft.setPosition(LEFT_INTAKE);
        dright.setPosition(RIGHT_INTAKE);
    }
}
