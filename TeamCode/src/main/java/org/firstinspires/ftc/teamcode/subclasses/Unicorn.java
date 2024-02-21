package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Unicorn extends Subsystem{
    protected Servo delivery;
    protected MecanumBotConstant m;
    private ElapsedTime timer;

    private double servoTimer1 = 0;
    private double servoTimer2 = 0;

    private boolean left_toggle = false;
    private boolean right_toggle = false;

    public double servoDelay = .5;
    public double rightTimer = 0;
    public double leftTimer = 0;


    public static double DELIVER = 0;
    public static double STOW_AWAY = 1;
    public static double RIGGING = .6;


    private static double delivery_position = 0;

    Telemetry telemetry;
    public Unicorn(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;


        delivery = hardwareMap.get(Servo.class, m.delivery);
        timer = new ElapsedTime();
    }



    public void deliver(){
        delivery.setPosition(DELIVER);
        update_position();
    }

    public void rigging(){
        delivery.setPosition(RIGGING);
        update_position();
    }
    public void stow(){
        delivery.setPosition(STOW_AWAY);
        update_position();
    }
    public void move_slowly_to(double position, double increments, int sleep) throws InterruptedException {
        update_position();
        if (delivery_position == position){
            return;
        }
        boolean direction = position > delivery_position;
        increments *= direction ? 1 : -1;

        for (double temp_position = delivery_position; temp_position < temp_position + 100*increments; temp_position += increments){
            if (direction && temp_position > position){
                break;
            }
            if (!direction && temp_position < position){
                break;
            }
            delivery.setPosition(temp_position);
            sleep(sleep);
        }
        delivery.setPosition(position);
    }
    public void update_position(){
        delivery_position = delivery.getPosition();
    }
    @Override
    public void telemetry() {
        telemetry.addData("Delivery Position: ", delivery_position);
    }
    public void goToPosition(double position){
        delivery_position = position;
        delivery.setPosition(position);
    }

    @Override
    public void init() {
        stow();


    }
}
