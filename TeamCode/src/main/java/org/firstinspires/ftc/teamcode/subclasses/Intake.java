package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;


@Config
public class Intake extends Subsystem{
    protected Servo rotation;
    protected Servo plunger;
    protected PIDMotor slide_rotation;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;


    private boolean update_rotation = true;

    public static int rotation_pickup = 0;
    public static double servo_rotation_pickup = 0.66;

    public static double plunger_in = .2;
    public static double plunger_one = .26;
    public static double plunger_two = .26;

    private static int rotation_init = 0;

    private static double truss_position = 0.15;
    private static double delivery_position = 0.17;

    public static double P = 0.0015, I = 0.0002, D = 0;
    public static double ROLLER_SPEED;
    private int delivery_stage = 0;
    //rotation intake position = 0.65
    //rotation parallel to slides = 0.70
    //rotation intake delivery = 1
    //the arm powers are backwarsd btu we are not fixing it. hahaha




    public static Servo.Direction SERVO_DIRECTION = Servo.Direction.REVERSE;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;
        slide_rotation = new PIDMotor(hardwareMap, telemetry, m.slides_rotation_motor);
        rotation = hardwareMap.get(Servo.class, m.intake_rotation_servo);
        rotation.setDirection(SERVO_DIRECTION);
        plunger = hardwareMap.get(Servo.class, m.plunger);
        timer = new ElapsedTime();
    }

    public void movePlate(double position){
        rotation.setPosition(position);
    }

    public void moveRotationTo(double position){
        slide_rotation.move_async(position);
    }
    public void movePlunger(double position){
        plunger.setPosition(position);
    }
    public void delivery_next(){
        if (delivery_stage > 3){
            delivery_stage = 1;
        }
        switch(delivery_stage){
            case 1:
                plunger.setPosition(plunger_in);
                delivery_stage += 1;
            case 2:
                plunger.setPosition(plunger_one);
                delivery_stage += 1;
            case 3:
                plunger.setPosition(plunger_two);
                delivery_stage = 1;

        }
    }

    public void update(){
        if (!update_rotation) {
            slide_rotation.move_async(slide_rotation.getCurrentPosition());
        }else{
            slide_rotation.update();
        }
    }
    public void rotate_bucket(){
        rotation.setPosition(.55);
    }

    public void go_to_transfer(){
        rotation.setPosition(truss_position);
        moveRotationTo(rotation_pickup);
        movePlunger(plunger_in);
    }

    public void setPower(double power){
        if(power == 0){
            update_rotation = true;
            return;
        }
        update_rotation = false;
        slide_rotation.setPower(power);
    }

    public void pickup(){
        moveRotationTo(rotation_pickup);
        movePlate(servo_rotation_pickup);
        movePlunger(0.46);

    }

    public void delivery(){
        moveRotationTo(2200);
        rotation.setPosition(delivery_position);
        movePlunger(plunger_in);
        delivery_stage = 2;

    }


    @Override
    public void telemetry() {
        slide_rotation.telemetry();
        telemetry.addData("Running", running);
    }

    @Override
    public void init() {
        slide_rotation.init();
        timer.reset();
        slide_rotation.setMin(0);
        slide_rotation.setMax(3000);
        slide_rotation.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_rotation.P = P;
        slide_rotation.I = I;
        slide_rotation.D = D;
        slide_rotation.move_sync(rotation_init, 5, .1);
        movePlate(truss_position);
        movePlunger(plunger_two);
    }

}
