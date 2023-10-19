package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class PlaneLauncher {
    DcMotorSimple launcher;
    MecanumBotConstant mc = new MecanumBotConstant();
    public PlaneLauncher(HardwareMap hardwareMap){
        launcher = hardwareMap.get(DcMotorSimple.class, mc.launcher_servo);
        launcher.setPower(1);
    }
    public void launch(){
        launcher.setPower(0);
    }

}
