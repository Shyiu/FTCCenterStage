package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.autonomous_utilities.MathFunctions;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;



public class AprilTagLocalization {

    public Pose2d getPosition(AprilTagDetection detection) {

            if (detection == null) {
                return new Pose2d(100, 100);
            }

            Pose2d aprilTagPose = new Pose2d(100, 100);
            if(detection.id == 1){
                aprilTagPose = new Pose2d(60,41);
            }else if(detection.id == 2){
                aprilTagPose = new Pose2d(60, 35);
            }else if(detection.id == 3){
                aprilTagPose = new Pose2d(60, 29);
            }else if(detection.id == 4){
                aprilTagPose = new Pose2d(60,-29);
            }else if(detection.id == 5){
                aprilTagPose = new Pose2d(60, -35);
            }else if(detection.id == 6){
                aprilTagPose = new Pose2d(60, -41);
            }
            double bearing = detection.ftcPose.bearing;
            double dy = detection.ftcPose.yaw;
            double dx = detection.ftcPose.range;
            Pose2d finalPose = new Pose2d(aprilTagPose.getX() - dx, aprilTagPose.getY() - dy, MathFunctions.AngleWrapPos(Math.toRadians(bearing) + Math.toRadians(180)))
            return finalPose;
        }
    }