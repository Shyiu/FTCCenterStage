package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous_utilities.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

@Autonomous(name = "Auto")
public class MecanumAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0.0,0.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
//        allPoints.add(new CurvePoint(0.0,120.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
//        allPoints.add(new CurvePoint(120.0,120.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            RobotMovement.goToLastPosition(0,30,.5, Math.toRadians(90), .5);
            robot.update();
            telemetry.addData("Heading", robot.getWorldAngle_rad());
            telemetry.addData("X Position", Robot.worldXPosition);
            telemetry.addData("Y Position", Robot.worldYPosition);
            telemetry.update();
        }
    }
}
