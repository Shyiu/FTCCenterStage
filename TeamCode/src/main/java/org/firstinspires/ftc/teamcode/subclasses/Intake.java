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
public class Intake extends Subsystem{
    protected Servo turn1;
    protected Servo turn2;
    protected DcMotor arm;
    protected DcMotorSimple counter_roller;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;

    public double servoTimer = 0;
    public double servoDelay = .3;

    public static double kf = -0.15;
    private static double SERVO_TWO_OFFSET = 0.03;
    private static double SERVO_ONE_OFFSET = 0;
    private static double MAX_SERVO = .6;
    public static double MOTOR_SPEED = 1;
    public static double servo_speed = .05; //This value is the rate of change at maximum input speed (degrees)
    public static double dt = .01;//This value is the loop time for the rate of change (seconds)
    public static double move_timer = 0;
    public static double servo_position1;
    public static double servo_position2;

    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;
        turn1 = hardwareMap.get(Servo.class, m.servo1);
        turn2 = hardwareMap.get(Servo.class, m.servo2);
        arm = hardwareMap.get(DcMotor.class, m.intake);
        counter_roller = hardwareMap.get(DcMotorSimple.class, m.counter_roller);
        servo_position1 = turn1.getPosition();
        servo_position2 = turn2.getPosition();
        turn1.setDirection(Servo.Direction.REVERSE);
        timer = new ElapsedTime();
    }
    public void toggle(){
        if (running && timer.time() - servoTimer > servoDelay){
            arm.setPower(0);
            counter_roller.setPower(0);
            servoTimer = timer.time();
            running = false;
        }else if (timer.time() - servoTimer > servoDelay){
            arm.setPower(MOTOR_SPEED);
            counter_roller.setPower(1);
            servoTimer = timer.time();
            running = true;
        }
    }
    public void setServoPower(double power){
        if (timer.time() - move_timer > dt) {
            servo_position1 -= power * servo_speed;
            servo_position2 -= power * servo_speed;
            servo_position1 = Math.min(MAX_SERVO - Math.max(SERVO_TWO_OFFSET, SERVO_ONE_OFFSET), servo_position1) + SERVO_ONE_OFFSET;
            servo_position1 = Math.max(SERVO_ONE_OFFSET, servo_position1);

            servo_position2 = Math.min(MAX_SERVO - Math.max(SERVO_TWO_OFFSET, SERVO_ONE_OFFSET), servo_position2) + SERVO_TWO_OFFSET;
            servo_position2 = Math.max(SERVO_TWO_OFFSET, servo_position2);
            turn1.setPosition(servo_position1);
            turn2.setPosition(servo_position2);
            move_timer = timer.time();
        }
    }
    public void setServoPosition(double position){
        turn1.setPosition(position);
        turn2.setPosition(position);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Counter Roller Power", counter_roller.getPower());
        telemetry.addData("Servo Position 1", servo_position1);
        telemetry.addData("Servo Position 2", servo_position2);
        telemetry.addData("Running", running);
    }

    @Override
    public void init() {
        turn1.setPosition(SERVO_ONE_OFFSET);
        turn2.setPosition(SERVO_TWO_OFFSET);
    }
}
