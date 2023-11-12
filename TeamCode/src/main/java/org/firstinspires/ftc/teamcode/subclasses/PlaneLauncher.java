package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class PlaneLauncher {
    Servo launcher;
    Servo rotation;
    MecanumBotConstant mc = new MecanumBotConstant();

    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, mc.launcher_servo);
        rotation = hardwareMap.get(Servo.class, mc.plane_servo);
        rotation.setPosition(0);
        launcher.setPosition(0);
    }
    public void launch(){
        launcher.setPosition(1);
    }
    public void reset(){
        launcher.setPosition(0);
    }

}
