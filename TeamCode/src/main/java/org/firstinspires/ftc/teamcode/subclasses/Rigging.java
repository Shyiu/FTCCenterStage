package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Rigging {
    DcMotor riggingMotor1;
    DcMotor riggingMotor2;
    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;

    private int extension_distance = 1680;//TODO: Find this value
    ElapsedTime timer;
    private double delay = .5;

    private double hold_speed = .15;


    public Rigging(HardwareMap hardwareMap){
        timer = new ElapsedTime();

        riggingMotor1 = hardwareMap.get(DcMotor.class, mc.rigging_motor_1);
        riggingMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotor2 = hardwareMap.get(DcMotor.class, mc.rigging_motor_2);
        riggingMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public boolean isBusy(){
        return busy;
    }
    public void rigUp(){
        if (first) {
            riggingMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            riggingMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            riggingMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            riggingMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            first = false;
            busy = true;
        }



        if (riggingMotor2.getCurrentPosition() > extension_distance || riggingMotor1.getCurrentPosition() < extension_distance) {
            setRiggingPower(0);
            busy = false;
        }
        else{
            setRiggingPower(1);
        }

    }
    public void rigDown(){
        if(riggingMotor2.getCurrentPosition() > 0 || riggingMotor1.getCurrentPosition() > 0){
            setRiggingPower(hold_speed);
            busy = false;
        }
        else{
            setRiggingPower(-1);
            busy = true;
        }
    }
    public void setRiggingPower(double speed){
        riggingMotor2.setPower(speed);
        riggingMotor1.setPower(speed);
    }
}
