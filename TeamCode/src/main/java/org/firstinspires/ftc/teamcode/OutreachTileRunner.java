package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;


import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp
public class OutreachTileRunner extends LinearOpMode {

    MecaTank mecatank;







    public MecanumBotConstant config = new MecanumBotConstant();

    @Override
    public void runOpMode() throws InterruptedException {





        mecatank = new MecaTank(hardwareMap, telemetry);



        mecatank.init();








        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            mecatank.setPowers(gamepad1.left_stick_y/-2.0, gamepad1.right_stick_y/-2.0,0,0);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }

    }


    public double sameSignSqrt(double number){
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }

}


