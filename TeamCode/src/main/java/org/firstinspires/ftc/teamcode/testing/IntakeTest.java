package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.MecanumTeleOp;
import org.firstinspires.ftc.teamcode.autonomous_utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;

@Config
@TeleOp(name = "servo_testing")
public class IntakeTest extends LinearOpMode {

    protected Servo turn_left;
    protected Servo turn_right;
    protected DcMotor arm;
    protected DcMotorSimple counter_roller;
    protected MecanumBotConstant m;
    public static double SERVO_POSITION = .5;
    public static double COUNTER_ROLLER_SPEED = -1;
    public static double SERVO_TWO_OFFSET = 0.03;
    public static double SERVO_ONE_OFFSET = 0;
    public static double MOTOR_SPEED = .8;

    public static boolean roller = false;
    public static boolean motor = false;
    protected MecaTank mecatank;
    protected Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m = new MecanumBotConstant();

        intake = new Intake(hardwareMap, telemetry);
        mecatank = new MecaTank(hardwareMap, telemetry);


        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            intake.set_speed((gamepad1.a || motor) ? MOTOR_SPEED : 0, (gamepad1.b || roller) ? COUNTER_ROLLER_SPEED : 0);

            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            intake.setServoPosition(SERVO_POSITION);

            intake.telemetry();
            mecatank.telemetry();
            telemetry.update();

        }
    }
    public double sameSignSqrt(double number) {
            return Math.copySign(Math.sqrt(Math.abs(number)), number);}
}
