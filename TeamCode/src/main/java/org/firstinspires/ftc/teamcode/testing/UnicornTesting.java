package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subclasses.Unicorn;

@Config
@TeleOp
public class UnicornTesting extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();
    protected Unicorn unicorn;


    public static double MAX_SPEED = .9;

    ElapsedTime timer = new ElapsedTime();



    double leftTgtPower = 0, rightTgtPower = 0;

    public MecanumBotConstant names = new MecanumBotConstant();
    public static double POSITION = 0;
    public static boolean MOVE_SLOWLY = false;
    public static double INCREMENT = 0.1;
    public static int SLEEP = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        unicorn = new Unicorn(hardwareMap, telemetry);

        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {
            if(MOVE_SLOWLY){
                unicorn.move_slowly_to(POSITION, INCREMENT, SLEEP);
                MOVE_SLOWLY = false;
            }else {
                unicorn.goToPosition(POSITION);
            }
            unicorn.telemetry();
            telemetry.update();
        }


//        public double closerToV2(double v1, double v2, double v3){
//            double diff1 = Math.abs(v1-v2);
//            double diff2 = Math.abs(v2-v3);
//            if (diff1 > diff2){
//                return v1;
//            }
//            return v3;
//        }

    }
}


