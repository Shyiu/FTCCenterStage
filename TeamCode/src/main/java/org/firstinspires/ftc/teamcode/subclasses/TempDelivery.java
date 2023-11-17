package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class TempDelivery {
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


    public static double RIGHT_OPEN = 1;
    public static double RIGHT_CLOSE = 1;
    public static double downPosition = .1;
    public TempDelivery(HardwareMap hardwareMap){
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
        main.setPosition(RIGHT_OPEN);
    }
    public void dropPixel() throws InterruptedException {
        main.setPosition(downPosition);
        sleep(800);
        setIn();
    }


}
