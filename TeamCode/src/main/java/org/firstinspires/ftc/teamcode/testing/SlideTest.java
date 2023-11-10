package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.Lift;
@Config
@TeleOp
public class SlideTest extends LinearOpMode {

    DcMotor slide;
    Lift lift;
    MecanumBotConstant m = new MecanumBotConstant();
    public static double P = 0.08;
    public static double I = 0;
    public static double D = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotor.class, m.slides);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = new Lift(hardwareMap, P ,I ,D);
        lift.reset();
        lift.moveTo(500);
        waitForStart();

        while(opModeIsActive()) {
            lift.update();
        }


    }
}
