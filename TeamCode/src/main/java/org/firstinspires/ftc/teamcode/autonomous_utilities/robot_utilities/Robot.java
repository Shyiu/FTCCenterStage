package org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

import java.util.concurrent.TimeUnit;

public class Robot {
    public static double xSpeed = 0.0D;
    public static double ySpeed = 0.0D;
    public static double turnSpeed = 0.0D;
    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;
    private Orientation angles;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Encoder deadwheelLateral;
    private Encoder deadwheelLinear;
    private double lateralXOffset = 0; //in cm (dist to center of robot)
    private double linearYOffset = 0; //in cm (dist to center of robot)

    private long previous_time_ns = 0;
    private ElapsedTime nano_timer;
    private int previousX = 0;
    private int previousY = 0;

    private static double TICKS_PER_REV = 537.6;
    private static double WHEEL_RADIUS = 1;
    private static double GEAR_RATIO = 1;

    private long lastUpdateTime = 0L;

    public Robot(HardwareMap hardwareMap, DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl, Encoder lateral, Encoder linear) {
        nano_timer = new ElapsedTime();
        worldXPosition = lateralXOffset;
        worldYPosition = linearYOffset;
        worldAngle_rad = Math.toRadians(90.0D);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        frontLeft =fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        deadwheelLateral = lateral;
        deadwheelLinear = linear;
        imu.initialize(parameters);
    }
    public long getTime(){
        return nano_timer.time(TimeUnit.NANOSECONDS);
    }
    public double getXPos() {
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }

    public double getWorldAngle_rad() {
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public double ticksToCM(double ticks){
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public void updatePosition(){
        long dt = getTime() - lastUpdateTime;
        if(dt > 50000){
            double lateralXComponent = deadwheelLateral.getCurrentPosition() * Math.cos(worldAngle_rad);
            double lateralYComponent = deadwheelLateral.getCurrentPosition() * Math.sin(worldAngle_rad);
            double linearXComponent = deadwheelLinear.getCurrentPosition() * Math.cos(worldAngle_rad);
            double linearYComponent = deadwheelLinear.getCurrentPosition() * Math.sin(worldAngle_rad);
            worldXPosition += Math.round(lateralXComponent + linearXComponent);
            worldYPosition += Math.round(lateralYComponent + linearYComponent);
            deadwheelLateral.resetEncoder();
            deadwheelLinear.resetEncoder();
            nano_timer.reset();
            lastUpdateTime = getTime();
        }
    }
    public double[] getCurrentPosition(){
        return new double[]{worldXPosition, worldYPosition};
    }
    public void setMotorPowers(double fl, double fr, double bl, double br){
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
    public void update() {
        double y = MovementVars.movement_y; // Remember, Y stick value is reversed
        double x = MovementVars.movement_x;
        double rx = MovementVars.movement_turn;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double botHeading = angles.firstAngle; //Z or the vertical axis in theroy need to check

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        updatePosition();
        worldAngle_rad = getWorldAngle_rad();
    }


}
