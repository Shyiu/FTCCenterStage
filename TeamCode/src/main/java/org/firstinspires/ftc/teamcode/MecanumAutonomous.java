package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous_utilities.CurvePoint;
import org.firstinspires.ftc.teamcode.autonomous_utilities.RobotMovement;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;

import java.util.ArrayList;

@Autonomous(name = "Auto")
public class MecanumAutonomous extends LinearOpMode {
    public MecanumBotConstant names = new MecanumBotConstant();


    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private Encoder deadwheelLateral;
    private Encoder deadwheelLinear;
    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);

        deadwheelLateral = new Encoder(hardwareMap, frontRight);
        deadwheelLinear = new Encoder(hardwareMap, frontLeft);

        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        Robot robot = new Robot(hardwareMap, frontRight, frontLeft, backRight,  backLeft, deadwheelLateral, deadwheelLinear);

//        ArrayList<CurvePoint> allPoints = new ArrayList<>();
//        allPoints.add(new CurvePoint(0.0,0.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
//        allPoints.add(new CurvePoint(0.0,120.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
//        allPoints.add(new CurvePoint(120.0,120.0,.5,0.5,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            RobotMovement.goToLastPosition(0,120,.5, Math.toRadians(90), .5);
            robot.update();
            telemetry.addData("Heading", robot.getWorldAngle_rad());
            telemetry.addData("X Position", Robot.worldXPosition);
            telemetry.addData("Y Position", Robot.worldYPosition);
            telemetry.update();
        }
    }
}
