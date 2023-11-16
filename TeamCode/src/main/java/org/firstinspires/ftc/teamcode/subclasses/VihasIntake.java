package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class VihasIntake {
    protected Servo turn1;
    protected DcMotor arm;
    protected DcMotorSimple counter_roller;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;

    public double servoTimer = 0;
    public double servoDelay = .3;

    public static double SERVO_START = 1;
    public static double SERVO_CLOSED = .23;
    public static double SERVO_OPEN = .4;

    public VihasIntake(HardwareMap hardwareMap){
        m = new MecanumBotConstant();

        turn1 = hardwareMap.get(Servo.class, m.servo1);
        turn1.setDirection(Servo.Direction.REVERSE);
        turn1.setPosition(SERVO_START);
        timer = new ElapsedTime();
    }
    public void toggle(){
        if (running && timer.time() - servoTimer > servoDelay){
            servoTimer = timer.time();
            open();
            running = false;
        }else if (timer.time() - servoTimer > servoDelay){
            servoTimer = timer.time();
            close();
            running = true;
        }
    }
    public void open(){
        turn1.setPosition(SERVO_OPEN);
        running = false;
    }
    public void close(){
        turn1.setPosition(SERVO_CLOSED);
        running = true;
    }

}
