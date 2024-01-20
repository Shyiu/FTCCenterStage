package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.SpinDelivery;
import org.firstinspires.ftc.teamcode.subclasses.THEBOOT;

@Disabled
@TeleOp(name="Spin Delivery Testing")
@Config
public class SpinDeliveryTest extends LinearOpMode {
    SpinDelivery delivery;
    @Override
    public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            delivery = new SpinDelivery(hardwareMap, telemetry);
            delivery.init();
            waitForStart();
            while(!isStopRequested() && opModeIsActive()) {
                if(gamepad1.x){
                    delivery.nextStage();
                }
                delivery.telemetry();
                telemetry.update();
            }

        }
    }

