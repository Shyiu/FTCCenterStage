package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
@Config
public class PlaneLauncher extends Subsystem{
    Servo launcher;
    private double startPos = 0.1;
    public static double launchPos = .21;
    private double flat = .3;
    MecanumBotConstant mc = new MecanumBotConstant();

    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, mc.plane_launcher_pinion_servo);

        launcher.setPosition(0);
    }

    public static double spring_stop_position = .07;
    public static double release_position = .4;
    public static double launch_position = .1;
    public static double stow_position = .75;



    public void moveLauncher(double position){
        launcher.setPosition(position);
    }
    public void launch(){
        launcher.setPosition(0);
    }
    public void reset(){
        launcher.setPosition(.65);
    }

    @Override
    public void init(){
        reset();
    }

    @Override
    public void telemetry(){
        return;
    }

}
