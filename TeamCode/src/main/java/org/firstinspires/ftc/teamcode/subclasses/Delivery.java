package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Delivery {
    protected DcMotorSimple dleft;
    protected DcMotorSimple dright;
    protected MecanumBotConstant m;
    private ElapsedTime timer;

    private double servoTimer1 = 0;
    private double servoTimer2 = 0;

    private boolean left_toggle = false;
    private boolean right_toggle = false;

    public double servoDelay = .5;


    public static double RIGHT_OPEN = 1;
    public static double LEFT_OPEN = 1;
    public Delivery(HardwareMap hardwareMap){
        m = new MecanumBotConstant();
        dleft = hardwareMap.get(DcMotorSimple.class, m.dleft);
        dright = hardwareMap.get(DcMotorSimple.class, m.dright);
        timer = new ElapsedTime();
    }
    public void toggleRight(){
        if (right_toggle && timer.time() - servoTimer1 > servoDelay){
            dright.setPower(0);
            servoTimer1 = timer.time();
            right_toggle = false;
        }else if (timer.time() - servoTimer1 > servoDelay){
            dright.setPower(RIGHT_OPEN);
            servoTimer2 = timer.time();
            left_toggle = true;
        }
    }
    public void toggleLeft(){
        if (left_toggle && timer.time() - servoTimer2 > servoDelay){
            dleft.setPower(0);
            servoTimer2 = timer.time();
            left_toggle = false;
        }else if (timer.time() - servoTimer2 > servoDelay){
            dleft.setPower(LEFT_OPEN);
            servoTimer2 = timer.time();
            left_toggle = true;
        }
    }

}
