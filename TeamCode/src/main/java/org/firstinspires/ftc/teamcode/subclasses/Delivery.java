package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Delivery {
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


    public static double RIGHT_OPEN = 1;
    public static double LEFT_OPEN = 1;
    public static double RIGHT_CLOSE = 0;
    public static double LEFT_CLOSE = 0;
    public Delivery(HardwareMap hardwareMap){
        m = new MecanumBotConstant();

        RIGHT_OPEN = m.right_open_position;
        LEFT_OPEN = m.left_open_position;
        RIGHT_CLOSE = m.right_closed_position;
        LEFT_CLOSE = m.left_closed_position;

        dleft = hardwareMap.get(Servo.class, m.dleft);
        dright = hardwareMap.get(Servo.class, m.dright);
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
            if(far(dleft.getPosition(), LEFT_CLOSE)) {
                dleft.setPosition(LEFT_CLOSE);
            }
        }else{
            if(far(dleft.getPosition(), LEFT_OPEN)) {
                dleft.setPosition(LEFT_OPEN);
            }
        }if (right_toggle){
            if(far(dright.getPosition(), RIGHT_CLOSE)) {
                dright.setPosition(RIGHT_CLOSE);
            }
        }else{
            if(far(dright.getPosition(), RIGHT_OPEN)) {
                dright.setPosition(RIGHT_OPEN);
            }
        }
    }


}
