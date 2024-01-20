package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
@Config
public class PlaneLauncher extends Subsystem{
    Servo launcher;
    Servo rotation;
    private double startPos = 0.1;
    public static double launchPos = .21;
    private double flat = .3;
    MecanumBotConstant mc = new MecanumBotConstant();

    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, mc.plane_launcher_spring_servo);
        rotation = hardwareMap.get(Servo.class, mc.plane_launcher_rotation_servo);
        rotation.setPosition(startPos);
        launcher.setPosition(0);
    }
    public void moveTo(double position){
        rotation.setPosition(position);
    }

    public void setFlatPosition(){
        rotation.setPosition(flat);
    }

    public void setStartPos(){
        rotation.setPosition(startPos);
    }

    public void setLaunchPosition(){
        rotation.setPosition(launchPos);
    }

    public static double spring_stop_position = .07;
    public static double release_position = .4;
    public static double launch_position = .1;
    public static double stow_position = .75;



    public void rotateLauncher(double position){
        rotation.setPosition(position);
    }
    public void moveLauncher(double position){
        launcher.setPosition(position);
    }
    public void rotate_to_launch(){
        rotation.setPosition(launch_position);
    }
    public void launch(){
        launcher.setPosition(1);
    }
    public void stow(){
        rotation.setPosition(stow_position);
    }
    public void reset(){
        launcher.setPosition(0);
    }

    @Override
    public void init(){
        setStartPos();
        stow();
        reset();
    }

    @Override
    public void telemetry(){
        return;
    }

}
