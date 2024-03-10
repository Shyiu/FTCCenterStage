package org.firstinspires.ftc.teamcode.roadrunner;

import android.nfc.NdefRecord;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DataTransfer;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subclasses.Color;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;
import org.firstinspires.ftc.teamcode.subclasses.Unicorn;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class PixelDrop extends LinearOpMode {
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
    public static SIDE side = SIDE.RIGHT;
    AUTO_STATES auto_states;
    Color color;
    Unicorn unicorn;
    Distance distance;
    ShivaniRigging rigging;
    private float[] colors;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        auto_states = AUTO_STATES.FIRST_PATH;

        timer = new ElapsedTime();


        unicorn = new Unicorn(hardwareMap, telemetry);
        distance = new Distance(hardwareMap, telemetry);
        color = new Color(hardwareMap, telemetry);

        unicorn.init();
        unicorn.travel();
        distance.init();
        color.init();
        timer.reset();


        mc = new MecanumBotConstant();
        drive = new MecanumDrive(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        Pose2d robotStart = new Pose2d(45,40, Math.toRadians(180));
        drive.setPoseEstimate(robotStart);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        double strafe_power = side.equals(SIDE.RIGHT) ? -0.4 : 0.4;
        while(!isStopRequested() && opModeIsActive()){
            drive.update();
            switch(auto_states){
                case FIRST_PATH:
                    if (!drive.isBusy()) {
                            if (!drive.isBusy()) {
                                rigging.setHookPower(0);
                                sleep(1000);
                                Pose2d startPose = drive.getPoseEstimate();
                                intoBackdrop = drive.trajectorySequenceBuilder(startPose)
                                        .back(getAdjustedDistance(),
                                                drive.getVelocityConstraint(DriveConstants.MAX_VEL * .10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                                        )
                                        .build();
                                drive.followTrajectorySequence(intoBackdrop);
                                strafe_timer = timer.time();
                                colors = color.getRGBValues();
                                if(colors[0] < 0.014){
                                    deliver();
                                    return;
                                }
                                drive.setWeightedDrivePower(new Pose2d(0, strafe_power));

                                auto_states = AUTO_STATES.IDENTIFY_SPOT;
                            }
                        }

                    break;
                case IDENTIFY_SPOT:
                    colors = color.getRGBValues();
                    telemetry.addData("red", colors[0]);
                    telemetry.addData("green", colors[1]);
                    telemetry.addData("blue", colors[2]);
                    if(colors[0] < 0.014 || timer.seconds() - strafe_timer > 3){
                        drive.setWeightedDrivePower(new Pose2d());
                        if(strafe_power > 0) {
                            intoBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .back(getAdjustedDistance(),
                                            drive.getVelocityConstraint(DriveConstants.MAX_VEL * .10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();
                        }
                        else{
                            intoBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .strafeRight(2)
                                    .back(getAdjustedDistance(),
                                            drive.getVelocityConstraint(DriveConstants.MAX_VEL * .10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();
                        }
                        drive.followTrajectorySequence(intoBackdrop);
                        deliver();
                        return;
                    }
                    break;



            }
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

    public void deliver(){
//        sleep(500);
        unicorn.deliver();
        sleep(1400);
        unicorn.goToPosition(0.45);
        sleep(500);
        unicorn.deliver();
        sleep(500);
    }
}
