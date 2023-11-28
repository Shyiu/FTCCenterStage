package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subclasses.Lift;
import org.firstinspires.ftc.teamcode.subclasses.THEBOOT;

public class FullSlideTesting extends LinearOpMode {
    Lift lift;
    THEBOOT boot;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, telemetry);
        boot = new THEBOOT(hardwareMap);
        lift.init();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            lift.setPower(-gamepad1.left_stick_y);
            if(gamepad1.a){
                ;
            }
        }
    }
}
