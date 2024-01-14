package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Rigging extends Subsystem{
    DcMotor riggingMotor;
    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;

    private int extension_distance = 14400;//TODO: Find this value
    ElapsedTime timer;
    private double delay = .5;

    private double hold_speed = 0;
    Telemetry telemetry;


    public Rigging(HardwareMap hardwareMap, Telemetry telemetry){
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        riggingMotor = hardwareMap.get(DcMotor.class, mc.rigging_motor_right);
        riggingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }
    public boolean isBusy(){
        return busy;
    }
    public void rigUp(){
        if (first) {
            riggingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            riggingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            first = false;
            busy = true;
            setRiggingPower(1);

        }
        if (getRiggingPosition() > extension_distance) {
            setRiggingPower(0);
            busy = false;
        }


    }
    public void rigDown(){
        if(getRiggingPosition() > 1000){
            setRiggingPower(hold_speed);
            busy = false;
        }
        else{
            setRiggingPower(-1);
            busy = true;
        }
    }
    public int getRiggingPosition(){

        return riggingMotor.getCurrentPosition();
    }
    public void setRiggingPower(double speed){
        riggingMotor.setPower(speed);
    }
    public void setMotorRightPower(double speed){
        riggingMotor.setPower(speed);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Position", getRiggingPosition());
        telemetry.addData("Busy", isBusy());
    }

    @Override
    public void init() {
        riggingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
