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
import org.firstinspires.ftc.teamcode.subclasses.Delivery;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;

@Config
@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {


    protected MecanumBotConstant m;

    public static double MOTOR_SPEED = 0;
    public static double SLIDES_SPEED = 0;
    public static double ARM_POSITION = 0;

    public static boolean roller = false;
    public static boolean motor = false;
    public static boolean reverse = false;
    public static boolean pid = false;
    public static boolean change = false;


    public static int target = -1000;
    protected MecaTank mecatank;
    protected Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m = new MecanumBotConstant();

        intake = new Intake(hardwareMap, telemetry);
        mecatank = new MecaTank(hardwareMap, telemetry);
        intake.init();


        intake.telemetry();
        mecatank.telemetry();
        telemetry.update();

        intake.moveRotationTo(target);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            if (gamepad1.a || roller){
                intake.forward_rollers();
            }else if(reverse){
                intake.reverse_rollers();
            }else{
                intake.disable_rollers();
            }

            intake.moveSlides(SLIDES_SPEED==0 ? sameSignSqrt(gamepad1.right_stick_y) : SLIDES_SPEED);
            intake.rotateSlides(MOTOR_SPEED==0 ? sameSignSqrt(gamepad1.left_stick_y) : MOTOR_SPEED);
            intake.moveRoller(ARM_POSITION);
            if (pid){
                intake.update();
            }
            if(change){
                intake.moveRotationTo(target);
                change = false;
            }




            intake.telemetry();
            mecatank.telemetry();
            telemetry.update();

        }
    }
    public double sameSignSqrt(double number) {
            return Math.copySign(Math.sqrt(Math.abs(number)), number);}
}
