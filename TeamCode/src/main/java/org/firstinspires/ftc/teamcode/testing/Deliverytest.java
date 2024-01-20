package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subclasses.TempDelivery;

@Disabled
@TeleOp(name="Servo Drop Testing")
@Config
public class Deliverytest extends LinearOpMode {
    TempDelivery delivery ;

    @Override
    public void runOpMode() throws InterruptedException {
            delivery = new TempDelivery(hardwareMap);
            waitForStart();
            while(!isStopRequested() && opModeIsActive()) {
                if(gamepad1.a) {
                    delivery.dropPixel();
                    sleep(1000);
                }
                if(gamepad1.y){
                    delivery.setUp();
                    sleep(1000);
                }
            }
            telemetry.update();
        }
    }

