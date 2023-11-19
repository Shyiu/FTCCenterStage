package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Rigging extends Subsystem{
    DcMotor riggingMotorRight;
    DcMotor riggingMotorLeft;
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
        riggingMotorRight = hardwareMap.get(DcMotor.class, mc.rigging_motor_right);
        riggingMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotorLeft = hardwareMap.get(DcMotor.class, mc.rigging_motor_left);
        riggingMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotorLeft.setDirection(DcMotor.Direction.REVERSE);

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
        int output = -riggingMotorLeft.getCurrentPosition() + riggingMotorRight.getCurrentPosition();
        if (riggingMotorRight.getCurrentPosition() == 0){
            output = -riggingMotorLeft.getCurrentPosition();
        }
        if(riggingMotorLeft.getCurrentPosition() == 0){
            output = riggingMotorRight.getCurrentPosition();
        }
        output /= 2;
        return output;
    }
    public int getRiggingLeft(){
        return riggingMotorLeft.getCurrentPosition();
    }
    public int getRiggingRight(){
        return riggingMotorRight.getCurrentPosition();
    }
    public void setRiggingPower(double speed){
        riggingMotorLeft.setPower(speed);
        riggingMotorRight.setPower(speed);
    }
    public void setMotorRightPower(double speed){
        riggingMotorRight.setPower(speed);
    }
    public void setMotorLeftPower(double speed){
        riggingMotorLeft.setPower(speed);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Left Position", getRiggingLeft());
        telemetry.addData("Right Position", getRiggingRight());
        telemetry.addData("Rigging Position", getRiggingPosition());
        telemetry.addData("Busy", isBusy());
    }

    @Override
    public void init() {
        riggingMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        riggingMotorLeft.setDirection(DcMotor.Direction.REVERSE);
    }
}
