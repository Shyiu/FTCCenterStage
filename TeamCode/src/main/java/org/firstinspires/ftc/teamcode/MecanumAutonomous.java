package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous_utilities.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.MovementVars;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

@Disabled
@Autonomous(name = "Auto")
public class MecanumAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, telemetry);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0.0,0.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
//        allPoints.add(new CurvePoint(0.0,120.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
//        allPoints.add(new CurvePoint(120.0,120.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            RobotMovement.goToPosition(0,120,.5, Math.toRadians(90), .5);
            robot.GFupdate();
            telemetry.addData("Heading", robot.getWorldAngle_rad());
            telemetry.addData("X Position", Robot.worldXPosition);
            telemetry.addData("Y Position", Robot.worldYPosition);
            telemetry.addData("Move X", MovementVars.movement_x);
            telemetry.addData("Move Y", MovementVars.movement_y);
            telemetry.addData("Move Turn", MovementVars.movement_turn);
            telemetry.addData("Angle to Target", RobotMovement.absoluteAngleToTarget);
            telemetry.update();
        }
    }
}
