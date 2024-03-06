package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subclasses.Distance;

@Config
@Autonomous
public class RealignmentAfterCollision extends LinearOpMode {
    MecanumBotConstant mc;
    MecanumDrive drive;
    TrajectorySequence intoBackdrop;
    public static double BACKDROP_DISTANCE = 1;
    private double strafe_timer;

    ElapsedTime timer;

    private enum AUTO_STATES{
        FIRST_PATH,
        IDENTIFY_SPOT
    }
    private enum SIDE{
        LEFT,
        RIGHT,
        MIDDLE
    }
    public static boolean avoid_descore = false;
    public static boolean red = false;
    public static SIDE side = SIDE.RIGHT;
    Vector2d endPose =  new Vector2d(49,34);

    AUTO_STATES auto_states;

    Distance distance;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        auto_states = AUTO_STATES.FIRST_PATH;

        timer = new ElapsedTime();


        distance = new Distance(hardwareMap, telemetry);


        distance.init();
        timer.reset();


        mc = new MecanumBotConstant();
        drive = new MecanumDrive(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        Pose2d robotStart = new Pose2d(14,61, Math.toRadians(180));
        drive.setPoseEstimate(robotStart);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();

        while(!isStopRequested() && opModeIsActive()){

            //here is the gameplan
            //first determine if there is a colision with a robot by checking the distance sensor, if it a is a small number, chances are we have collided with a robot or atleast crossed the truss.
            //Then also, assuming the deadwheels are not trash, check the x position to see if we have crossed the truss. The order here does not matter to much tbh.
            //Then spline back and to 0 degrees and begin to strafe until the distance sensor gets something between like 10 and 50 inches (probably the backdrop)
            //Use a different
            if(drive.getPoseEstimate().getX() > 13){
                TrajectorySequenceBuilder toBackdropAdjusted;
                if(!red) {
                    toBackdropAdjusted = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(13, 48, Math.toRadians(180)), Math.toRadians(270));
                    if (avoid_descore) {
                        toBackdropAdjusted
                                .strafeTo(new Vector2d(13, 33))
                                .splineToConstantHeading(new Vector2d(12, 13), 0)
                                .splineToConstantHeading(new Vector2d(25, 13), 0)
                                .splineToConstantHeading(endPose, 0);
                    } else {
                        toBackdropAdjusted
                                .strafeTo(new Vector2d(13, 43))
                                .strafeTo(new Vector2d(16, 44))
                                .splineToConstantHeading(endPose, 0);
                    }
                }else{
                    toBackdropAdjusted = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(new Pose2d(13, -48, Math.toRadians(180)), Math.toRadians(90));
                    if (avoid_descore) {
                        toBackdropAdjusted
                                .strafeTo(new Vector2d(13, -33))
                                .splineToConstantHeading(new Vector2d(12, -13), 0)
                                .splineToConstantHeading(new Vector2d(25, -13), 0)
                                .splineToConstantHeading(endPose, 0);
                    } else {
                        toBackdropAdjusted
                                .strafeTo(new Vector2d(13, -43))
                                .strafeTo(new Vector2d(16, -44))
                                .splineToConstantHeading(endPose, 0);
                    }
                }
                drive.followTrajectorySequence(toBackdropAdjusted.build());
                //deliver();

            }
            drive.update();
            telemetry.update();


            }
        }

    public double getAdjustedDistance(){
        double output = distance.getDistFromRobotEdge()+ 0.05;
        if (output > 78){
            output = 8;
        }
        return output;

    }
}
