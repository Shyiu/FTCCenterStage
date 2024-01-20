package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;

@Config
@TeleOp
public class DriveTrainTesting extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();



    public MecanumBotConstant names = new MecanumBotConstant();


    private MecaTank mecatank;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        MecanumDrive drive = new MecanumDrive(hardwareMap);

        mecatank = new MecaTank(hardwareMap, telemetry);


        mecatank.init();
        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {
            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            mecatank.telemetry();
            drive.update();
            telemetry.update();
        }
    }
}


