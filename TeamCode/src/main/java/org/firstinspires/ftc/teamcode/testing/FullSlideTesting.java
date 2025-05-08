package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.Lift;

@Config
@TeleOp
public class FullSlideTesting extends LinearOpMode {
    Lift lift;
    public static boolean pid = false;
    public static int target_position = 400;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, telemetry);
        lift.init();
        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            lift.setPower(-gamepad1.left_stick_y);
            lift.moveTo(target_position);
            if (pid){
                lift.update();
            }
            lift.telemetry();
            telemetry.update();
        }
    }
}
