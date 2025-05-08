package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.Distance;

@Config
@TeleOp(name = "DistanceTesting")
public class DistanceTesting extends LinearOpMode {
    Distance right_distance;

    Distance left_distance;
    Distance front_distance;

    public static double front_filter = 0.8;
    public static double left_filter = 0.8;
    public static double right_filter = 0.8;

    public static boolean testing_with_trizzy = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        right_distance = new Distance(hardwareMap,telemetry, true);
        front_distance   = new Distance(hardwareMap,telemetry);
        if(!testing_with_trizzy) {
            left_distance = new Distance(hardwareMap, telemetry, false);
        }

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if(!testing_with_trizzy) {
                telemetry.addData("Filtered Left Distance", left_distance.getFilteredDist());
                left_distance.setFilter(left_filter);
                left_distance.update();
                left_distance.telemetry();
            }

            telemetry.addData("Filtered Right Distance", right_distance.getFilteredDist());

            right_distance.setFilter(right_filter);

            right_distance.update();

            right_distance.telemetry();

            telemetry.addData("Filtered Front Distance", front_distance.getFilteredDist());

            front_distance.setFilter(front_filter);

            front_distance.update();

            front_distance.telemetry();
            telemetry.update();
        }
    }

}
