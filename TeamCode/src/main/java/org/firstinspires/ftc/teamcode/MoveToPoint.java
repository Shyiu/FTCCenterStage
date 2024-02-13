package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous_utilities.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.MovementVars;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

@Disabled
@Autonomous(name = "PurelyPursuiting")
public class MoveToPoint extends LinearOpMode {
    @Override
    public void runOpMode() {

        MecanumDrive robot = new MecanumDrive(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (!isStopRequested() && opModeIsActive())
        {
            RobotMovement.goToPosition(10,10,.5, Math.toRadians(90),Math.toRadians(30));
            robot.update();
            robot.followPurePursuit();
            telemetry.addData("Expected Velo", MovementVars.expected_velocity);
            telemetry.addData("Actual Velo", robot.getExternalHeadingVelocity());
            telemetry.addData("X Position", robot.getPoseEstimate().getX());
            telemetry.addData("Y Position", robot.getPoseEstimate().getX());
            telemetry.update();
        }
    }
}
