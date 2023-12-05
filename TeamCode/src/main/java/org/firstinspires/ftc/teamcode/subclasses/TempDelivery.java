package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class TempDelivery {
    protected Servo axle_rotation_servo;
    protected MecanumBotConstant config;
    private ElapsedTime timer;

    private double servoTimer1 = 0;
    private double servoTimer2 = 0;

    private boolean left_toggle = false;
    private boolean right_toggle = false;

    public double servoDelay = .5;
    public double rightTimer = 0;
    public double leftTimer = 0;


    public static double RIGHT_OPEN = 1;
    public static double RIGHT_CLOSE = 1;
    public static double downPosition = .05;
    public TempDelivery(HardwareMap hardwareMap){
        config = new MecanumBotConstant();
        axle_rotation_servo = hardwareMap.get(Servo.class, config.servo2);
        axle_rotation_servo.setDirection(Servo.Direction.FORWARD);
        setUp();
    }
    public void moveTo(double position){
        axle_rotation_servo.setPosition(position);
    }
    public void setIn(){
        axle_rotation_servo.setPosition(RIGHT_CLOSE);
    }
    public void setUp(){
        axle_rotation_servo.setPosition(RIGHT_OPEN);
    }
    public void dropPixel() throws InterruptedException {
//        for (double i = main.getPosition(); i >= downPosition; i-=0.1){
//            main.setPosition(i);
//            sleep(20);
//        }
        axle_rotation_servo.setPosition(downPosition);
        sleep(2000);
        setIn();
    }
    public void dropPixelSlowly() throws InterruptedException{
        for (double i = axle_rotation_servo.getPosition(); i >= downPosition; i-=0.1){
            axle_rotation_servo.setPosition(i);
            sleep(20);
        }
        axle_rotation_servo.setPosition(downPosition);
        sleep(800);
        setIn();
    }


}
