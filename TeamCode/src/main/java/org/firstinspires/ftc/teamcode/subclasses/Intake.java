package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Intake extends Subsystem{
    private static final double COUNTER_ROLLER_SPEED = -1;
    protected Servo turn_left;
    protected Servo turn_right;
    protected DcMotor arm;
    protected DcMotorSimple counter_roller;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;

    public double servoTimer = 0;
    public double servoDelay = .3;

    public static double kf = -0.15;
    public static double SERVO_TWO_OFFSET = 0;
    public static double SERVO_ONE_OFFSET = 0;
    private static double MAX_SERVO = .6;
    public static double MOTOR_SPEED = 1;
    public static double servo_speed = .05; //This value is the rate of change at maximum input speed (degrees)
    public static double dt = .01;//This value is the loop time for the rate of change (seconds)
    public static double move_timer = 0;
    public static double servo_position1;
    public static double servo_position2;
    private static double[] stack_positions = new double[]{.56};
    private static double start_position = .50;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;
        turn_left = hardwareMap.get(Servo.class, m.servo_left);
        turn_right = hardwareMap.get(Servo.class, m.servo_right);
        turn_left.setDirection(Servo.Direction.REVERSE);
        arm = hardwareMap.get(DcMotor.class, m.intake_motor);
        counter_roller = hardwareMap.get(DcMotorSimple.class, m.counter_roller);
        counter_roller.setDirection(DcMotorSimple.Direction.REVERSE);
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
            counter_roller.setPower(COUNTER_ROLLER_SPEED);
            servoTimer = timer.time();
            running = true;
        }
    }
    public void set_speed(double MOTOR_SPEED, double COUNTER_ROLLER_SPEED){
        if (timer.time() - servoTimer > servoDelay){
            arm.setPower(MOTOR_SPEED);
            counter_roller.setPower(COUNTER_ROLLER_SPEED);
            servoTimer = timer.time();
        }
    }
    public void deliver_pixel() throws InterruptedException {
        set_speed(-0.2, 1);
        sleep(500);
        set_speed(0,0);
    }
    public void setServoPower(double power){
        if (timer.time() - move_timer > dt) {
            servo_position1 -= power * servo_speed;
            servo_position2 -= power * servo_speed;
            servo_position1 = Math.min(MAX_SERVO - Math.max(SERVO_TWO_OFFSET, SERVO_ONE_OFFSET), servo_position1) + SERVO_ONE_OFFSET;
            servo_position1 = Math.max(SERVO_ONE_OFFSET, servo_position1);

            servo_position2 = Math.min(MAX_SERVO - Math.max(SERVO_TWO_OFFSET, SERVO_ONE_OFFSET), servo_position2) + SERVO_TWO_OFFSET;
            servo_position2 = Math.max(SERVO_TWO_OFFSET, servo_position2);
            turn_left.setPosition(servo_position1);
            turn_right.setPosition(servo_position2);
            move_timer = timer.time();
        }
    }
    public void setServoPosition(double position){
        position = Math.min(1 - SERVO_TWO_OFFSET, position);
        position = Math.max(-1, position);
        turn_left.setPosition(position + SERVO_ONE_OFFSET);
        turn_right.setPosition(position + SERVO_TWO_OFFSET);
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
        setServoPosition(start_position);
    }
}
