package org.firstinspires.ftc.teamcode.subclasses;

import android.telecom.TelecomManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Intake {
    protected Servo turn1;
    protected Servo turn2;
    protected DcMotor arm;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;

    public double servoTimer = 0;
    public double servoDelay = .3;

    public static double kf = -0.15;

    public static double MOTOR_SPEED = 1;
    public static double servo_speed = .05; //This value is the rate of change at maximum input speed (degrees)
    public static double dt = .01;//This value is the loop time for the rate of change (seconds)
    public static double move_timer = 0;
    public static double servo_position1;
    public static double servo_position2;
    public Intake(HardwareMap hardwareMap){
        m = new MecanumBotConstant();

        turn1 = hardwareMap.get(Servo.class, m.servo1);
        turn2 = hardwareMap.get(Servo.class, m.servo2);
        arm = hardwareMap.get(DcMotor.class, m.intake);
        servo_position1 = turn1.getPosition();
        servo_position2 = turn2.getPosition();
        timer = new ElapsedTime();
    }
    public void toggle(){
        if (running && timer.time() - servoTimer > servoDelay){
            arm.setPower(0);
            servoTimer = timer.time();
            running = false;
        }else if (timer.time() - servoTimer > servoDelay){
            arm.setPower(MOTOR_SPEED);
            servoTimer = timer.time();
            running = true;
        }
    }
    public void setServoPower(double power){
        if (timer.time() - move_timer > dt) {
            servo_position1 += power * servo_speed;
            servo_position2 -= power * servo_speed;
            servo_position1 = Math.min(1, servo_position1);
            servo_position2 = Math.max(-1, servo_position2);
            turn1.setPosition(servo_position1);
            turn2.setPosition(servo_position2);
            move_timer = timer.time();
        }
    }
}
