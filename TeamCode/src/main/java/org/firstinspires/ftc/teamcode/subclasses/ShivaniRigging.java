package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class ShivaniRigging extends Subsystem{
    PIDMotor riggingMotor;
    DcMotorSimple left_servo, right_servo;
    MecanumBotConstant mc = new MecanumBotConstant();
    private boolean busy = false;
    private boolean first = true;
    private boolean up = false;
    double error, lastError;

    public static double P = 8, I = 0, D = 0;
    public static double target_position = -3000;
    ElapsedTime timer;
    private double delay = .5;

    public static double hold_speed = 0.15;
    private String state = "Init";
    Telemetry telemetry;
    public static double LEFT_OPEN_POWER = .6, RIGHT_OPEN_POWER = .4;
    public static double LEFT_CLOSE_POWER = .45, RIGHT_CLOSE_POWER = .55;

    //>0.5 brings the left arm up and <0.5 brings the right arm up

    public ShivaniRigging(HardwareMap hardwareMap, Telemetry telemetry){
        timer = new ElapsedTime();
        this.telemetry = telemetry;
        riggingMotor = new PIDMotor(hardwareMap, telemetry, mc.rigging_motor);
        riggingMotor.setMin(-3000);





        left_servo = hardwareMap.get(DcMotorSimple.class, mc.rigging_left);
        right_servo = hardwareMap.get(DcMotorSimple.class, mc.rigging_right);
    }

    public void open(){
        left_servo.setPower(LEFT_OPEN_POWER);
        right_servo.setPower(RIGHT_OPEN_POWER);
        state = "Extended";
    }
    public void open_left(){

        state = "Left Opening";

        left_servo.setPower(LEFT_OPEN_POWER);

    }
    public void open_right(){
        state = "Right Opening";
        right_servo.setPower(RIGHT_OPEN_POWER);
    }

    public void close_left(){
        state = "Left Retracting";
        left_servo.setPower(LEFT_CLOSE_POWER);

    }
    public void close_right(){
        state = "Right Retracting";
        right_servo.setPower(RIGHT_CLOSE_POWER);
    }
    public void freeze_right(){
        state = "Right Frozen";
        right_servo.setPower(0.5);
    }
    public void freeze_left(){
        state = "Left Frozen";
        left_servo.setPower(0.5);
    }
    public void extend_rigging(){
        activate();
        //TODO: Extend the arms or something.
    }
    public void close(){
        state = "Retracted";
        left_servo.setPower(LEFT_CLOSE_POWER);
        right_servo.setPower(RIGHT_CLOSE_POWER);
    }
    public double getArmPositions(int arm){

        return arm == 0 ? left_servo.getPower() : right_servo.getPower();
    }
    public void setRiggingPower(double speed){
        riggingMotor.setPower(speed + hold_speed);
    }

    public void holdRiggingMotors(){
        riggingMotor.setPower(hold_speed);
    }
    @Override
    public void telemetry() {
        riggingMotor.telemetry();
        telemetry.addData("State: ", state);

    }
    public boolean isBusy(){
        return up ? riggingMotor.getCurrentPosition() < target_position : riggingMotor.getCurrentPosition() > target_position;
    }
    public void activate(){
        riggingMotor.moveTo(target_position);
    }

    public void update(){
        riggingMotor.update();
    }
    public void raise_rigging_motor(){
        riggingMotor.bringToHalt();
    }
    @Override
    public void init() {
        riggingMotor.init();
        riggingMotor.P = P;
        riggingMotor.I = I;
        riggingMotor.D = D;
    }
}
