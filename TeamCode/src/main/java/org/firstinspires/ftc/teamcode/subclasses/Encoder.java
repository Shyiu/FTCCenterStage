package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Encoder {
    DcMotor encoder;
    int reverse = 1;


    private int lastPosition;
    private double lastUpdateTime;

    public Encoder(HardwareMap hardwareMap, DcMotor motor){
        encoder = motor;
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastUpdateTime = 0;
        lastPosition = 0;
    }
    public void resetEncoder(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastUpdateTime = 0;
        lastPosition = 0;
    }
    public void setReverse(boolean state){
        if (state){
            reverse = -1;
        }else{
            reverse =1;
        }
    }
    public int getCurrentPosition(){
        return encoder.getCurrentPosition() * reverse;
    }
}
