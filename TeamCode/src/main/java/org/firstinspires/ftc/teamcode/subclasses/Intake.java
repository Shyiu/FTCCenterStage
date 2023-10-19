package org.firstinspires.ftc.teamcode.subclasses;

import android.telecom.TelecomManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Intake {
    protected DcMotorSimple turn1;
    protected DcMotorSimple turn2;
    protected DcMotor arm;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;

    public double servoTimer = 0;
    public double servoDelay = .5;

    public static double kf = -0.15;

    public static double SERVO_SPEED = 1;
    public Intake(HardwareMap hardwareMap){
        m = new MecanumBotConstant();
        turn1 = hardwareMap.get(DcMotorSimple.class, m.servo1);
        turn2 = hardwareMap.get(DcMotorSimple.class, m.servo2);
        arm = hardwareMap.get(DcMotor.class, m.intake);
        timer = new ElapsedTime();
    }
    public void toggle(){
        if (running && timer.time() - servoTimer > servoDelay){
            turn1.setPower(0);
            turn2.setPower(0);
            servoTimer = timer.time();
            running = false;
        }else if (timer.time() - servoTimer > servoDelay){
            turn1.setPower(SERVO_SPEED);
            turn2.setPower(-SERVO_SPEED);
            servoTimer = timer.time();
            running = true;
        }
    }
    public void setPower(double power){
        arm.setPower(power + kf);
    }
    public double getArmPower(){
        return arm.getPower();
    }

}
