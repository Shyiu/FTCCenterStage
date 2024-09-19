package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;

@Config
@TeleOp()
public class IntakeControllerTest extends LinearOpMode {


    protected MecanumBotConstant m;

    public static double PLATE_POSITION = 0;
    public static double PLUNGER_POSITION = 0;
    public static double TEST_DISTANCE = 8;


    public static boolean pid = false;
    public static boolean controller = false;

    public static double target_angle = 30;
    public static int target = -1000;

    protected Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m = new MecanumBotConstant();

        intake = new Intake(hardwareMap, telemetry);


        intake.init();


        intake.telemetry();
        telemetry.update();

        intake.moveArm(target);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            if (pid){
                intake.update();
            }
            intake.moveArm(target);
            if(controller){
                intake.moveBucketWithController(target_angle);
            }else{
                intake.moveBucket(PLATE_POSITION);
            }






            intake.telemetry();

            telemetry.update();

        }
    }
    public double sameSignSqrt(double number) {
            return Math.copySign(Math.sqrt(Math.abs(number)), number);}
}
