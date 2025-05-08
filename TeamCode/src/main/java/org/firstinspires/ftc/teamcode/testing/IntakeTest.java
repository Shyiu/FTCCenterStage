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
@TeleOp(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {


    protected MecanumBotConstant m;

    public static double PLATE_POSITION = 0;
    public static double PLUNGER_POSITION = 0;
    public static double TEST_DISTANCE = 8;

    public static boolean motor = false;
    public static boolean reverse = false;
    public static boolean pid = false;
    public static boolean change = false;
    public static boolean compensation = false;
    public static boolean old_compensation = true;
    public static boolean auto_move_arm = false;

    public static int target = -1000;

    protected MecaTank mecatank;
    protected Intake intake;
    protected Distance distance;
    protected Distance front_distance;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m = new MecanumBotConstant();

        intake = new Intake(hardwareMap, telemetry);
        mecatank = new MecaTank(hardwareMap, telemetry);
        front_distance = new Distance(hardwareMap,telemetry, false);
        distance = new Distance(hardwareMap,telemetry, true);

        intake.init();
        distance.init();
        front_distance.init();


        intake.telemetry();
        mecatank.telemetry();
        telemetry.update();

        intake.moveArm(target);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            if (pid){
                intake.update();
            }
            if(change){
                intake.moveArm(target);
                change = false;
            }
            if(compensation){
                intake.bucket_compensation();
            }else{
                intake.moveBucket(PLATE_POSITION);

            }

            if(old_compensation){
                intake.useOldAlignment();
            }else{
                intake.useCurrentAlignment();

            }

            if(auto_move_arm){
                int distance_reading = intake.calculate_arm_limit(distance.getFilteredDist());
                intake.moveArm(distance_reading);
            }
            telemetry.addData("Arm Limit Test", intake.calculate_arm_limit(TEST_DISTANCE));
            telemetry.addData("Arm Desired Position Test", intake.calculate_arm_limit(distance.getFilteredDist()));
            intake.calculate_robot_distance_limit();
            intake.moveClutch(PLUNGER_POSITION);
            intake.setPower(-gamepad1.left_stick_y);


            intake.telemetry();
            mecatank.telemetry();
            distance.telemetry();
            distance.update();
            front_distance.update();
            front_distance.telemetry();   

            telemetry.update();

        }
    }
    public double sameSignSqrt(double number) {
            return Math.copySign(Math.sqrt(Math.abs(number)), number);}
}
