package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class PlaneLauncher extends Subsystem{
    Servo launcher;
    public static double spring_stop_position = .65;
    public static double release_position = 0;
    MecanumBotConstant mc = new MecanumBotConstant();

    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, mc.plane_launcher_pinion_servo);


    }


    public void launch(){
        launcher.setPosition(release_position);
    }

    public void reset(){
        launcher.setPosition(spring_stop_position);
    }

    @Override
    public void init(){
        launcher.setPosition(spring_stop_position);
    }

    @Override
    public void telemetry(){
        return;
    }

}
