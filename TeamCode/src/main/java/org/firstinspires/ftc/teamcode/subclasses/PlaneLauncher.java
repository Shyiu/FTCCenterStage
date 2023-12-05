package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class PlaneLauncher extends Subsystem{
    Servo launcher;
    Servo rotation;
    public static double startPos = 0.1;
    public static double launchPos = .57;
    private double flat = .3;
    private double spring_stop_position = .6;
    private double release_position = 0;
    MecanumBotConstant mc = new MecanumBotConstant();

    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, mc.plane_launcher_pinion_servo);
        rotation = hardwareMap.get(Servo.class, mc.plane_launcher_rotation_servo);
        rotation.setPosition(startPos);
        launcher.setPosition(spring_stop_position);
    }
    public void moveTo(double position){
        rotation.setPosition(position);
    }

    public void setFlat(){
        rotation.setPosition(flat);
    }

    public void setStartPos(){
        rotation.setPosition(startPos);
    }

    public void setLaunchPos(){
        rotation.setPosition(launchPos);
    }

    public void launch(){
        launcher.setPosition(release_position);
    }

    public void reset(){
        launcher.setPosition(spring_stop_position);
    }

    @Override
    public void init(){
        setStartPos();
    }

    @Override
    public void telemetry(){
        return;
    }

}
