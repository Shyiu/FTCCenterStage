package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //14.25
                .setConstraints(44.13626700549987, 30, 2.4687686920166017, Math.toRadians(60), 17.22)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.5, 63, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(25.00, 38, Math.toRadians(90)))

                                .lineTo(new Vector2d(25, 49))




//                                .splineToSplineHeading(new Pose2d(49.00, 28.50, Math.toRadians(180.00)), Math.toRadians(360.00))



//                                .lineToLinearHeading(new Pose2d(15.00, 43.00, Math.oRadians(45)), Math.toRadians(90))
//                                .splineTo(new Vector2d(6.00, 32.00), Math.toRadians(45))
//                                .lineTo(new Vector2d(6.00, 32.00))
//                                .lineTo(new Vector2d(8.00, 34.00))
//                                .splineToSplineHeading(new Pose2d(49, 28.5, Math.toRadians(180)), Math.toRadians(0))


//                                .splineToSplineHeading(new Pose2d(49, -32, Math.toRadians(180)), Math.toRadians(0))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}