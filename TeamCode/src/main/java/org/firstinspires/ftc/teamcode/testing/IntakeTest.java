package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;

@Config
@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {


    protected MecanumBotConstant m;

    public static double PLATE_POSITION = 0;
    public static double PLUNGER_POSITION = 0;

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
            if (pid){
                intake.update();
            }
            if(change){
                intake.moveRotationTo(target);
                change = false;
            }
            intake.movePlate(PLATE_POSITION);
            intake.movePlunger(PLUNGER_POSITION);
            intake.setPower(-gamepad1.left_stick_y);


            intake.telemetry();
            mecatank.telemetry();
            telemetry.update();

        }
    }
    public double sameSignSqrt(double number) {
            return Math.copySign(Math.sqrt(Math.abs(number)), number);}
}
