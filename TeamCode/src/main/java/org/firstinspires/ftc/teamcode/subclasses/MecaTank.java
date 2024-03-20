package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class MecaTank extends Subsystem{
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Distance distance_rear_left;
    private Distance distance_rear_right;
    private Distance distance_front;
    private MecanumBotConstant config;
    private Telemetry telemetry;
    private double MAX_DRIVE_SPEED = 1;
    public static double REAR_MIN_DISTANCE = 2.08;
    public static double FRONT_MIN_DISTANCE = 5.7;
    public static double distance_sensor_distance = 7.5;
    private boolean enable_front_stop = false;
    private boolean enable_rear_stop = false;
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry){
        config = new MecanumBotConstant();
        frontLeft = hardwareMap.get(DcMotor.class, config.fl);
        frontRight = hardwareMap.get(DcMotor.class, config.fr);
        backLeft = hardwareMap.get(DcMotor.class, config.bl);
        backRight = hardwareMap.get(DcMotor.class, config.br);

        distance_rear_left = new Distance(hardwareMap, telemetry, false);
        distance_rear_right = new Distance(hardwareMap, telemetry, true);
        distance_front = new Distance(hardwareMap, telemetry);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.telemetry = telemetry;

    }
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry, Distance distance_rear_right, Distance distance_rear_left, Distance distance_front){
        config = new MecanumBotConstant();
        frontLeft = hardwareMap.get(DcMotor.class, config.fl);
        frontRight = hardwareMap.get(DcMotor.class, config.fr);
        backLeft = hardwareMap.get(DcMotor.class, config.bl);
        backRight = hardwareMap.get(DcMotor.class, config.br);

        this.distance_rear_left = distance_rear_left;
        this.distance_rear_right = distance_rear_right;
        this.distance_front = distance_front;

        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.telemetry = telemetry;

    }
    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    public void set_min_distance(double distance){
        REAR_MIN_DISTANCE = distance;
    }
    public void set_individual_powers(double fl_power, double fr_power, double bl_power, double br_power){
        frontRight.setPower(fr_power);
        frontLeft.setPower(fl_power);
        backLeft.setPower(bl_power);
        backRight.setPower(br_power);
    }
    public double calculate_angle(){
        double x1 = distance_rear_right.getFilteredDist();
        double x2 = distance_rear_left.getFilteredDist();
        double delta_x = x2 - x1;
        return Math.toDegrees(Math.atan(delta_x/distance_sensor_distance));
    }
    public void setPowers(double left_stick_y, double right_stick_y, double left_trigger, double right_trigger){


        if (left_trigger != 0) {
            double posPower = sameSignSqrt(left_trigger);
            double negPower = sameSignSqrt(-left_trigger);
            frontLeft.setPower(negPower * MAX_DRIVE_SPEED);
            backRight.setPower(negPower * MAX_DRIVE_SPEED);
            frontRight.setPower(posPower * MAX_DRIVE_SPEED);
            backLeft.setPower(posPower * MAX_DRIVE_SPEED);
            return;

        } else if (right_trigger != 0) {
            double posPower = sameSignSqrt(right_trigger);
            double negPower = sameSignSqrt(-right_trigger);
            frontLeft.setPower(posPower * MAX_DRIVE_SPEED);
            backRight.setPower(posPower * MAX_DRIVE_SPEED);
            frontRight.setPower(negPower * MAX_DRIVE_SPEED);
            backLeft.setPower(negPower * MAX_DRIVE_SPEED);
            return;
        }

        double leftPower = sameSignSqrt(-left_stick_y);
        double rightPower = sameSignSqrt(-right_stick_y);

        if(enable_front_stop && rightPower > 0 && leftPower > 0 && distance_front.getFilteredDist() < FRONT_MIN_DISTANCE) {
            frontRight.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            return;
        }
        if (enable_rear_stop && leftPower < 0 && distance_rear_left.getFilteredDist() < REAR_MIN_DISTANCE){
            frontLeft.setPower(0);
            backLeft.setPower(0);
        }else{
            frontLeft.setPower(leftPower * MAX_DRIVE_SPEED);
            backLeft.setPower(leftPower * MAX_DRIVE_SPEED);
        }
        if(enable_rear_stop &&  rightPower < 0 && distance_rear_right.getFilteredDist() < REAR_MIN_DISTANCE){
            frontRight.setPower(0);
            backRight.setPower(0);
        }else {
            frontRight.setPower(rightPower * MAX_DRIVE_SPEED);
            backRight.setPower(rightPower * MAX_DRIVE_SPEED);
        }







    }

    public void setFrontStop(boolean stop) {
        enable_front_stop = stop;
    }
    public void setRearStop(boolean stop){
        enable_rear_stop = stop;
    }
        @Override
    public void telemetry() {

        telemetry.addData("Front Right Motor Power", frontRight.getPower());
        telemetry.addData("Front Left Motor Power", frontLeft.getPower());
        telemetry.addData("Back Right Motor Power", backRight.getPower());
        telemetry.addData("Back Left Motor Power", backLeft.getPower());
    }

    @Override
    public void init() {

    }

    public void setMaxPower(double v) {
        MAX_DRIVE_SPEED = v;
    }
}
