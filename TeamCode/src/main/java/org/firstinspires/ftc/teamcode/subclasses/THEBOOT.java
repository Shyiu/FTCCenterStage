package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class THEBOOT {
    protected Servo boot_rotation_servo;
    protected MecanumBotConstant config;
    private ElapsedTime timer;

    private double servoTimer1 = 0;
    private double servoTimer2 = 0;

    private boolean left_toggle = false;
    private boolean right_toggle = false;

    public double servoDelay = .5;
    public double rightTimer = 0;
    public double leftTimer = 0;

    public static double RIGHT_CLOSE = .675;
    public static double downPosition = .15;
    public static double FLAT_POSITION = .5;

    public THEBOOT(HardwareMap hardwareMap){
        config = new MecanumBotConstant();
        boot_rotation_servo = hardwareMap.get(Servo.class, config.servo_right);
        boot_rotation_servo.setDirection(Servo.Direction.FORWARD);
        setUp();
    }

    public void moveTo(double position){
        boot_rotation_servo.setPosition(position);
    }

    public void setIn(){
        boot_rotation_servo.setPosition(RIGHT_CLOSE);
    }

    public void setUp(){
        boot_rotation_servo.setPosition(FLAT_POSITION);
    }

    public void dropPixel() throws InterruptedException {
        boot_rotation_servo.setPosition(downPosition);
        sleep(2000);
        setIn();
    }
    public void setPosition(double position)  throws InterruptedException {
        boot_rotation_servo.setPosition(position);
        sleep(200);
    }

    public void dropPixelSlowly() throws InterruptedException{
        for (double i = boot_rotation_servo.getPosition(); i >= downPosition; i-=0.1){
            boot_rotation_servo.setPosition(i);
            sleep(20);
        }
        boot_rotation_servo.setPosition(downPosition);
        sleep(800);
        setIn();
    }


}
