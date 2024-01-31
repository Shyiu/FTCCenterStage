//package org.firstinspires.ftc.teamcode.roadrunner;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//@Disabled
//public class PathFlipper {
//    right = drive.trajectorySequenceBuilder(new Pose2d(16.5, 63, Math.toRadians(90)))
//            .setReversed(true)
//            .splineToLinearHeading(new Pose2d(8.50, 34.5, Math.toRadians(45)), Math.toRadians(270))
//            .setReversed(false)
//            .splineToLinearHeading(new Pose2d(right_blue, Math.toRadians(180)), Math.toRadians(0))
//
//            .build();
//
//
//    middle = drive.trajectorySequenceBuilder(new Pose2d(16.5, 63, Math.toRadians(90)))
//            .setReversed(true)
//                        .splineToLinearHeading(new Pose2d(16.50, 30.5, Math.toRadians(90)), Math.toRadians(270))
//            .forward(3)
//                        .splineToConstantHeading(new Vector2d(25.50, 48.5), Math.toRadians(0))
//
//            .splineToConstantHeading(new Vector2d(32.50, 44.5), Math.toRadians(270))
//
//            .splineTo(middle_blue, Math.toRadians(0))
//
//            .build();
//
//    left = drive.trajectorySequenceBuilder(new Pose2d(16.5, 63, Math.toRadians(90)))
//
//            .setReversed(true)
//                        .splineToLinearHeading(new Pose2d(25.50, 36.5, Math.toRadians(90)), Math.toRadians(270))
//            .forward(3)
//                        .splineToConstantHeading(new Vector2d(25.50, 60.5), Math.toRadians(0))
//
//            .splineToConstantHeading(new Vector2d(32.50, 56.5), Math.toRadians(270))
//
//            .splineTo(left_blue, Math.toRadians(0))
//            .build();
//
//}
