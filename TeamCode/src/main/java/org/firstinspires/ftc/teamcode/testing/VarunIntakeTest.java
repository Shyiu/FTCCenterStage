package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.Delivery;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;
import org.firstinspires.ftc.teamcode.subclasses.VarunIntake;

@Config
@Disabled
@TeleOp(name = "VarunIntakeTest")
public class VarunIntakeTest extends LinearOpMode {


    protected MecanumBotConstant m;
    public static double SERVO_POSITION = .5;
    public static double COUNTER_ROLLER_SPEED = -1;
    public static double SERVO_TWO_OFFSET = 0.03;
    public static double SERVO_ONE_OFFSET = 0;
    public static double MOTOR_SPEED = .8;
    public static double DELIVERY_POSITION = 0;


    public static boolean roller = false;
    public static boolean motor = false;
    protected MecaTank mecatank;
    protected VarunIntake intake;
    protected Delivery delivery;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m = new MecanumBotConstant();

        intake = new VarunIntake(hardwareMap, telemetry);
        mecatank = new MecaTank(hardwareMap, telemetry);
        delivery = new Delivery(hardwareMap, telemetry);
        delivery.init();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            intake.set_speed((gamepad1.a || motor) ? MOTOR_SPEED : 0, (gamepad1.b || roller) ? COUNTER_ROLLER_SPEED : 0);

            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            intake.setServoPosition(SERVO_POSITION);

            delivery.goToPosition(DELIVERY_POSITION);

            intake.telemetry();
            delivery.telemetry();
            mecatank.telemetry();
            telemetry.update();

        }
    }
    public double sameSignSqrt(double number) {
            return Math.copySign(Math.sqrt(Math.abs(number)), number);}
}
