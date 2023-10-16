package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Encoder {
    DcMotor encoder;
    int reverse = 1;
    public Encoder(HardwareMap hardwareMap, DcMotor motor){
        encoder = motor;
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncoder(){
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setReverse(boolean state){
        if (!state){
            reverse = -1;
        }
    }
    public int getPosition(){
       return encoder.getCurrentPosition() * reverse;
    }
}
