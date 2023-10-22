package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Rigging {
    DcMotor riggingMotorRight;
    DcMotor riggingMotorLeft;
    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;

    private int extension_distance = 1680;//TODO: Find this value
    ElapsedTime timer;
    private double delay = .5;

    private double hold_speed = .15;


    public Rigging(HardwareMap hardwareMap){
        timer = new ElapsedTime();

        riggingMotorRight = hardwareMap.get(DcMotor.class, mc.rigging_motor_right);
        riggingMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotorLeft = hardwareMap.get(DcMotor.class, mc.rigging_motor_left);
        riggingMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotorRight.setDirection(DcMotor.Direction.REVERSE);

    }
    public boolean isBusy(){
        return busy;
    }
    public void rigUp(){
        if (first) {
            riggingMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            riggingMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            riggingMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            riggingMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            first = false;
            busy = true;
            setRiggingPower(1);

        }



        if (riggingMotorLeft.getCurrentPosition() > extension_distance || riggingMotorRight.getCurrentPosition() > extension_distance) {
            setRiggingPower(0);
            busy = false;
        }


    }
    public void rigDown(){
        if(riggingMotorLeft.getCurrentPosition() > 0 || riggingMotorRight.getCurrentPosition() > 0){
            setRiggingPower(hold_speed);
            busy = false;
        }
        else{
            setRiggingPower(-1);
            busy = true;
        }
    }
    public int getRiggingPosition(){
        int output = riggingMotorLeft.getCurrentPosition() + riggingMotorRight.getCurrentPosition();
        output /= 2;
        return output;
    }
    public void setRiggingPower(double speed){
        riggingMotorLeft.setPower(speed);
        riggingMotorRight.setPower(speed);
    }
}
