package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.MRDistance;

@Config
@TeleOp
public class MRDistanceTesting extends LinearOpMode {
    MRDistance distance;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        distance = new MRDistance(hardwareMap,telemetry);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            distance.telemetry();
            telemetry.update();
        }
    }

}
