package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;


@Config
public class Intake extends Subsystem{
    protected Servo rotation;
    protected Servo clutch;
    protected PIDMotor slide_rotation;
    protected MecanumBotConstant m;
    protected boolean running = false;
    private ElapsedTime timer;

    private boolean update_rotation = true;
    private boolean gyro_ready = false;

    public static int rotation_pickup = 0;
    public static double servo_rotation_pickup = 0.6;
    private static double anchor_position = 0;

    public static double clutch_in = 0;
    public static double clutch_one = .13;
    public static double clutch_two = .25;

//    public static double deg_per_tick = 90.0/(1700 - 603);//will replace with gyro
//    public static double level_position = 603;
//    public static double servo_moves_per_degree = 180/1.0;
//    public static double STARTING_ANGLE = 15;
//    public static double STARTING_ARM_HEIGHT = 9.875;

    private static int rotation_init = 0;

    private static double truss_position = 0.05;
    private static double delivery_position = 0.07;

    public static double P = 0.0015, I = 0.0002, D = 0;
    private int delivery_stage = 0;
    //rotation intake position = 0.65
    //rotation parallel to slides = 0.70
    //rotation intake delivery = 1


    private double ARM_LENGTH = 15.875;//in
    private double ARM_ANGLE = -30;
    private double BACKDROP_HEIGHT = 35;//in
    private double BACKDROP_LENGTH = 10.5;//in

    public static Servo.Direction SERVO_DIRECTION = Servo.Direction.REVERSE;
    Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        m = new MecanumBotConstant();
        this.telemetry = telemetry;
        slide_rotation = new PIDMotor(hardwareMap, telemetry, m.slides_rotation_motor);

        rotation = hardwareMap.get(Servo.class, m.bucket_servo);
        rotation.setDirection(SERVO_DIRECTION);
        clutch = hardwareMap.get(Servo.class, m.clutch);
        timer = new ElapsedTime();
    }

    public void moveBucket(double position){
        if(position <= 0.7 && position >= 0) {
            rotation.setPosition(position);
        }else if(position > 0.7){
            rotation.setPosition(0.7);
        }else{
            rotation.setPosition(0);
        }
        update_anchor();

    }
    public void moveBucket(double position, boolean update_pivot){
        if(position <= 0.9) {
            rotation.setPosition(position);
        }else{
            rotation.setPosition(0.9);
        }
        if(update_pivot) {
            update_anchor();
        }

    }

    public void moveArm(double position){
        slide_rotation.move_async(position);
    }
    public void moveClutch(double position){
        clutch.setPosition(position);
    }
    public void delivery_next(){
        if (delivery_stage > 3){
            delivery_stage = 1;
        }
        switch(delivery_stage){
            case 1:
                clutch.setPosition(clutch_in);
                delivery_stage += 1;
                break;
            case 2:
                clutch.setPosition(clutch_one);
                delivery_stage += 1;
                break;
            case 3:
                clutch.setPosition(clutch_two);
                delivery_stage = 1;
                break;

        }
    }

    public void update(){
        if (!update_rotation) {
            slide_rotation.move_async(slide_rotation.getCurrentPosition());
        }else{
            slide_rotation.update();
        }
//        ARM_ANGLE = 180 - ((slide_rotation.getCurrentPosition() - level_position) * deg_per_tick);


    }
    public void rotate_bucket(){

        rotation.setPosition(.45);
        update_anchor();
    }
    public void update_anchor(){
        anchor_position = rotation.getPosition();
    }
    public void go_to_transfer(){
        rotation.setPosition(truss_position);
        moveArm(rotation_pickup);
        moveClutch(clutch_in);
        update_anchor();

    }

    public void setPower(double power){
        if(power == 0){
            update_rotation = true;
            return;
        }
        update_rotation = false;
        slide_rotation.setPower(power);
    }

    public void pickup(){
        moveArm(rotation_pickup);
        moveBucket(servo_rotation_pickup);
        update_anchor();


    }
    public boolean isBusy(){
        return slide_rotation.isBusy();
    }
    public void raise_clutch(){
        moveClutch(0.46);
    }

    public void delivery(){
        moveArm(2200);
        rotation.setPosition(delivery_position);
        moveClutch(clutch_in);
        delivery_stage = 2;
        update_anchor();


    }


    @Override
    public void telemetry() {
        slide_rotation.telemetry();
        telemetry.addData("Anchor Position", anchor_position);
        telemetry.addData("Running", running);
        telemetry.addData("Arm Angle", ARM_ANGLE);
    }


    @Override
    public void init() {



//        if(!gyro_ready){
//            throw new AssertionError("Init Gyro");
//        }
        slide_rotation.init();
        timer.reset();
        slide_rotation.setMin(0);
        slide_rotation.setMax(2700);
        slide_rotation.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_rotation.P = P;
        slide_rotation.I = I;
        slide_rotation.D = D;
        slide_rotation.move_sync(rotation_init, 5, .1);
        moveBucket(truss_position);
        moveClutch(clutch_two);
    }
    public double calculate_robot_distance_limit(){
        double robot_dist = 0;
        if(slide_rotation.getCurrentPosition() > 2000) {
            double m = (5.4 - 2.3)/(2400 - 2200);
            double b = 3.3;
            robot_dist = m * (slide_rotation.getCurrentPosition() - 2300) + b;
           
        }
        telemetry.addData("Robot Distance", robot_dist);
        return robot_dist;
    }
    public double calculate_robot_distance_limit(boolean use_target_position){
        double robot_dist = 0;
        if(!use_target_position){
            return calculate_robot_distance_limit();
        }
        if(slide_rotation.targetPos > 2000) {
            double m = (5.4 - 2.3)/(2400 - 2200);
            double b = 3.3;
            robot_dist = m * (slide_rotation.getCurrentPosition() - 2300) + b;

        }
        telemetry.addData("Robot Distance", robot_dist);
        return robot_dist;
    }
    public void bucket_compensation(){
        if(slide_rotation.getCurrentPosition() > 2000) {
            double m = (0.09 - 0.05)/(2450 - 2300);
            double b = 0.05;
            double target = m * (slide_rotation.getCurrentPosition() - 2300) + b;
            moveBucket(target);
            telemetry.addData("Target", target);
        }


    }

    public void addBucketPos(double increase) {
        moveBucket(anchor_position + increase, false);
    }
}
