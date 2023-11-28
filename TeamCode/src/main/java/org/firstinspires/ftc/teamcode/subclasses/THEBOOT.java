package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class THEBOOT {
    protected Servo main;
    protected MecanumBotConstant m;
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
        m = new MecanumBotConstant();
        main = hardwareMap.get(Servo.class, m.servo2);
        main.setDirection(Servo.Direction.FORWARD);
        setUp();
    }

    public void moveTo(double position){
        main.setPosition(position);
    }

    public void setIn(){
        main.setPosition(RIGHT_CLOSE);
    }

    public void setUp(){
        main.setPosition(FLAT_POSITION);
    }

    public void dropPixel() throws InterruptedException {
        main.setPosition(downPosition);
        sleep(2000);
        setIn();
    }
    public void setPosition(double position)  throws InterruptedException {
        main.setPosition(position);
        sleep(200);
    }

    public void dropPixelSlowly() throws InterruptedException{
        for (double i = main.getPosition(); i >= downPosition; i-=0.1){
            main.setPosition(i);
            sleep(20);
        }
        main.setPosition(downPosition);
        sleep(800);
        setIn();
    }


}
