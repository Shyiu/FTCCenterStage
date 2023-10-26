package org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities;

//TODO: Determine what the starting IMU heading is and find if we need to offset by 90
//TODO: Figure out how the localization works.

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.MovementVars.movement_turn;
import static org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.MovementVars.movement_x;
import static org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.MovementVars.movement_y;

import android.os.SystemClock;
import android.sax.StartElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.autonomous_utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

import java.util.concurrent.TimeUnit;
@Config
public class Robot {

    public static double worldXPosition = 0;
    public static double worldYPosition = 0;
    public static double worldAngle_rad;

    public double firstAngle = 0;
    public double secondAngle = 0;
    public double thirdAngle = 0;

    private double xSpeed, ySpeed, turnSpeed;

    private Orientation angles;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Encoder deadwheelLateral;
    private Encoder deadwheelLinear;

    public double lastLateralX = 0;
    public double lastLateralY = 0;
    public double lastLinearX = 0;
    public double lastLinearY = 0;

    private double lateralXOffset = 0; //in cm (dist to center of robot)
    private double linearYOffset = 0; //in cm (dist to center of robot)

    private long dt = Long.MAX_VALUE;

    private MecanumBotConstant names = new MecanumBotConstant();

    private long previous_time_ns = 0;
    private ElapsedTime nano_timer;
    private int previousX = 0;
    private int previousY = 0;

    private static double TICKS_PER_REV = 8192;
    private static double WHEEL_RADIUS_CM = 1.75;
    private static double GEAR_RATIO = 1;

    private long lastUpdateTime = 0L;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        nano_timer = new ElapsedTime();
        worldXPosition = lateralXOffset;
        worldYPosition = linearYOffset;
        worldAngle_rad = Math.toRadians(90.0D);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;


        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        deadwheelLateral = new Encoder(hardwareMap, frontRight);
        deadwheelLinear = new Encoder(hardwareMap, frontLeft);

        deadwheelLateral.setReverse(true);
        deadwheelLinear.setReverse(false);


        imu.initialize(parameters);

        worldAngle_rad = getWorldAngle_rad();

        telemetry.addLine("IMU Calibrated");
        telemetry.update();

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
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double degree = MathFunctions.AngleWrapPos(angles.firstAngle + Math.toRadians(90));
        firstAngle = MathFunctions.AngleWrapPos(angles.firstAngle + Math.toRadians(90));
        secondAngle = MathFunctions.AngleWrapPos(angles.secondAngle + Math.toRadians(90));
        thirdAngle = MathFunctions.AngleWrapPos(angles.thirdAngle + Math.toRadians(90));

        return degree;
    }
    public double ticksToCM(double ticks){
        return WHEEL_RADIUS_CM * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public void updatePosition(){

        if(dt > 500000){
            double lateralXComponent = deadwheelLateral.getCurrentPosition() * Math.sin(worldAngle_rad);//untested
            double lateralYComponent = deadwheelLateral.getCurrentPosition() * Math.cos(worldAngle_rad);//untested
            double linearXComponent = deadwheelLinear.getCurrentPosition() * Math.cos(worldAngle_rad);
            double linearYComponent = deadwheelLinear.getCurrentPosition() * Math.sin(worldAngle_rad);
            worldXPosition += ticksToCM(lateralXComponent + linearXComponent);
            worldYPosition += ticksToCM(lateralYComponent  + linearYComponent);
            lastLinearX = linearXComponent;
            lastLinearY = linearYComponent;
            lastLateralX = lateralXComponent;
            lastLateralY = lateralYComponent;
            worldAngle_rad = getWorldAngle_rad();

            dt = getTime() - lastUpdateTime;
            deadwheelLinear.resetEncoder();
            deadwheelLateral.resetEncoder();
            nano_timer.reset();
            lastUpdateTime = getTime();
        }
    }
    public void drawRobot() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        fieldOverlay.strokeCircle(worldXPosition, worldYPosition, 9);

        dashboard.sendTelemetryPacket(packet);

    }
    public double[] getCurrentPosition(){
        return new double[]{worldXPosition, worldYPosition};
    }
    public void setMotorPowers(double right, double left) throws InterruptedException{
        frontLeft.setPower(left);
        frontRight.setPower(right);
        backLeft.setPower(left);
        backRight.setPower(right);
    }
    public double[] getPowers(){
        return new double[]{frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower()};
    }
    public void update() {
        double y = movement_y; // Remember, Y stick value is reversed
        double x = movement_x;
        double rx = movement_turn;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.

        double botHeading = MathFunctions.AngleWrapPos(getWorldAngle_rad() - Math.toRadians(90));

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
    }
    public void GFupdate(){
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;


        //figre out the signs with shreyas
        double tl_power_raw = movement_y-movement_turn +movement_x*1.1;
        double bl_power_raw = movement_y - movement_turn - movement_x*1.1;
        double br_power_raw = -movement_y- movement_turn- movement_x*1.1;
        double tr_power_raw = -movement_y- movement_turn + movement_x*1.1;


        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;


        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        frontLeft.setPower(tl_power_raw);
        backLeft.setPower(bl_power_raw);
        backRight.setPower(br_power_raw);
        frontRight.setPower(tr_power_raw);

    }


}
