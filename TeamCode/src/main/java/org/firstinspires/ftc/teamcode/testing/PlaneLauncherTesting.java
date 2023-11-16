package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;

@TeleOp(name="Plane Launcher Testing")
@Config
public class PlaneLauncherTesting extends LinearOpMode {
    Servo servo;
    Servo turn;
    MecanumBotConstant mc;
    ElapsedTime timer;
    double currentTime = 0;
    double primed = 0;
    public static double SERVO_POSITION = .06;
    double launch = 1;
    PlaneLauncher planeLauncher;
    @Override
    public void runOpMode(){
        mc = new MecanumBotConstant();
//        servo = hardwareMap.get(Servo.class, mc.launcher_servo);
//        turn = hardwareMap.get(Servo.class, mc.plane_servo);
        planeLauncher = new PlaneLauncher(hardwareMap);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("a to launch");
            telemetry.addLine("b to launch");

           if (gamepad1.a && timer.time() - currentTime > .5){
                planeLauncher.setLaunchPos();
                sleep(300);
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
