package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;

@TeleOp(name="Plane Launcher Testing")
@Config
public class PlaneLauncherTesting extends LinearOpMode {

    MecanumBotConstant config;
    ElapsedTime timer;
    double currentTime = 0;
    public static double SERVO_POSITION = .4;
    PlaneLauncher planeLauncher;
    @Override
    public void runOpMode(){
        config = new MecanumBotConstant();
        planeLauncher = new PlaneLauncher(hardwareMap);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("a to launch");
            telemetry.addLine("b to reset");
            planeLauncher.moveTo(SERVO_POSITION);
            if (gamepad1.a && timer.time() - currentTime > .5){
                planeLauncher.launch();
            }
            if (gamepad1.b && timer.time() - currentTime > .5){
                planeLauncher.reset();
                currentTime= timer.time();
            }
            telemetry.update();
        }
    }
}
