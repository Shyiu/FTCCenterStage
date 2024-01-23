package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

import java.util.concurrent.TimeUnit;


@Config
public class Intake extends Subsystem{
    protected Servo rotation;
    protected DcMotorSimple roller;
    public Lift slides;
    protected PIDMotor slide_rotation;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;
    private boolean going_up = true;
    private int target_position = 0;
    private ElapsedTime srvoTimer;
    public double servoDelay = .3;
    private boolean delivering = false;
    public static double MOTOR_SPEED = 1;
    public static double move_timer = 0;
    public static double servo_position1;
    public static double servo_position2;

    public static double holding_speed = -0.0005;

    public static int slide_pickup = 0;
    public static int rotation_pickup = 0;
    public static double servo_rotation_pickup = 0.6;

    private static double init_position = 1;
    private static int rotation_init = 0;

    private static double truss_position = 0.85;
    private static double delivery_position = 0.85;

    public static double P = 0.0005, I = 0.0002, D = 0, F = 0.0005;
    public static double ROLLER_SPEED;
    //rotation intake position = 0.65
    //rotation parallel to slides = 0.70
    //rotation intake delivery = 1
    //the arm powers are backwarsd btu we are not fixing it. hahaha




    public static DcMotorSimple.Direction SERVO_DIRECTION = DcMotorSimple.Direction.REVERSE;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;
        rotation = hardwareMap.get(Servo.class, m.intake_rotation_servo);
        roller = hardwareMap.get(DcMotorSimple.class, m.intake_servo);
        roller.setDirection(SERVO_DIRECTION);
        slides = new Lift(hardwareMap, telemetry);
        slide_rotation = new PIDMotor(hardwareMap, telemetry, m.slides_rotation_motor);
        slide_rotation.P = P;
        slide_rotation.I = I;
        slide_rotation.D = D;
        slide_rotation.F =  F;

        timer = new ElapsedTime();
    }
    public void moveSlides(double power){
        slides.setPower(power);
    }
    public void toggleRollers(){
        if (servoDelay > 500){
            roller.setPower(running ? 0 : ROLLER_SPEED);
            running = !running;
            servoDelay = timer.time(TimeUnit.MILLISECONDS);
        }
    }
    public void moveRoller(double position){
        rotation.setPosition(position);
    }
    public void rotateSlides(double power){
        slide_rotation.setPower(power);
    }

    public void moveSlidesTo(double position){
        slides.moveTo(position);
    }
    public void moveRotationTo(double position){
        slide_rotation.move_async(position);
    }

    public void update(){
        slides.update();
        slide_rotation.update();

    }
    public void go_under_truss(){
        rotation.setPosition(truss_position);
    }
    public void go_to_transfer(){
        rotation.setPosition(truss_position);
        moveSlidesTo(slide_pickup);
        moveRotationTo(rotation_pickup);
        ROLLER_SPEED = 1;
    }
    public void increaseRotation(double power){
        slide_rotation.setPower(slide_rotation.getPower() + power);
    }
    public void pickup(){
        moveSlidesTo(slide_pickup);
        moveRotationTo(rotation_pickup);
        moveRoller(servo_rotation_pickup);
        ROLLER_SPEED = 1;
    }

    public void delivery(){
        ROLLER_SPEED = -1;
        moveRotationTo(-2150);
        rotation.setPosition(delivery_position);

    }
    public void enable_rollers(){
        roller.setPower(ROLLER_SPEED);
    }
    public void forward_rollers(){
        roller.setPower(1);
    }
    public void disable_rollers(){
        roller.setPower(0);
    }
    public void reverse_rollers(){
        roller.setPower(-1);
    }


    @Override
    public void telemetry() {
        slides.telemetry();
        slide_rotation.telemetry();
        telemetry.addData("Running", running);
    }

    @Override
    public void init() {
        slide_rotation.init();
        timer.reset();
        slide_rotation.setMax(0);
        slide_rotation.setMin(-3600);
        moveSlides(0);
        disable_rollers();
        slide_rotation.move_sync(rotation_init, 5, .1);
        moveRoller(init_position);
        slides.init();
    }

}
