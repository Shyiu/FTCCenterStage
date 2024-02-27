package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class MecaTank extends Subsystem{
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Distance distance;
    private MecanumBotConstant config;
    private Telemetry telemetry;
    private final double MAX_DRIVE_SPEED = 1;
    public static double MIN_DISTANCE = 0.58;
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry){
        config = new MecanumBotConstant();
        frontLeft = hardwareMap.get(DcMotor.class, config.fl);
        frontRight = hardwareMap.get(DcMotor.class, config.fr);
        backLeft = hardwareMap.get(DcMotor.class, config.bl);
        backRight = hardwareMap.get(DcMotor.class, config.br);

        distance = new Distance(hardwareMap, telemetry);
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
        MIN_DISTANCE = distance;
    }
    public void set_individual_powers(double fl_power, double fr_power, double bl_power, double br_power){
        frontRight.setPower(fr_power);
        frontLeft.setPower(fl_power);
        backLeft.setPower(bl_power);
        backRight.setPower(br_power);
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
        if (leftPower < 0 && rightPower < 0 && distance.getDistFromRobotEdge() < MIN_DISTANCE){
            frontRight.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);
            return;
        }
        frontRight.setPower(rightPower * MAX_DRIVE_SPEED);
        backRight.setPower(rightPower * MAX_DRIVE_SPEED);

        frontLeft.setPower(leftPower * MAX_DRIVE_SPEED);
        backLeft.setPower(leftPower * MAX_DRIVE_SPEED);




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
}
