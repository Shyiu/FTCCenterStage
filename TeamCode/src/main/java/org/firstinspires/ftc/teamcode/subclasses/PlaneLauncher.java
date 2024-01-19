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

    public void launch(){
        launcher.setPosition(1);
    }

    public void reset(){
        launcher.setPosition(0);
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
