package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class PlaneLauncher {
    Servo launcher;
    MecanumBotConstant mc = new MecanumBotConstant();
    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(Servo.class, mc.launcher_servo);
        launcher.setPosition(1);
    }
    public void launch(){
        launcher.setPosition(0);
    }
    public void reset(){
        launcher.setPosition(1);
    }

}
