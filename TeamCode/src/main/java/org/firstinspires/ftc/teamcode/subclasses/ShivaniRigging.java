package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class ShivaniRigging extends Subsystem{
    DcMotor riggingMotor;
    DcMotorSimple left_servo, right_servo;
    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;

    private int extension_distance = 14400;//TODO: Find this value
    ElapsedTime timer;
    private double delay = .5;

    private double hold_speed = 0;
    private String state = "Init";
    Telemetry telemetry;
    public static double LEFT_OPEN_SERVO_POSITION = 0, RIGHT_OPEN_SERVO_POSITION = 0;
    public static double LEFT_CLOSED_SERVO_POSITION = 0, RIGHT_CLOSED_SERVO_POSITION = 0;

    //>0.5 brings the left arm up and <0.5 brings the right arm up

    public ShivaniRigging(HardwareMap hardwareMap, Telemetry telemetry){
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        riggingMotor = hardwareMap.get(DcMotor.class, mc.rigging_motor);
        riggingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_servo = hardwareMap.get(DcMotorSimple.class, mc.rigging_left);
        right_servo = hardwareMap.get(DcMotorSimple.class, mc.rigging_right);
    }

    public void extend(){
        left_servo.setPower(LEFT_OPEN_SERVO_POSITION);
        right_servo.setPower(RIGHT_OPEN_SERVO_POSITION);
        state = "Extended";
    }
    public void retract(){
        state = "Retracted";
        left_servo.setPower(LEFT_CLOSED_SERVO_POSITION);
        right_servo.setPower(RIGHT_CLOSED_SERVO_POSITION);
    }
    public int getRiggingPosition(){
        return riggingMotor.getCurrentPosition();
    }
    public double getArmPositions(int arm){

        return arm == 0 ? left_servo.getPower() : right_servo.getPower();
    }
    public void setRiggingPower(double speed){
        riggingMotor.setPower(speed);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Position", getRiggingPosition());
        telemetry.addData("State: ", state);

    }

    @Override
    public void init() {
        riggingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        riggingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        riggingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        retract();

    }
}
