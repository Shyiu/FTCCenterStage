package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.THEBOOT;
import org.firstinspires.ftc.teamcode.subclasses.TempDelivery;

@TeleOp(name="THE BOOT Testing")
@Config
public class THEBOOTTest extends LinearOpMode {
    THEBOOT delivery;
    public static double DEFAULT_POSITION = 0;
    @Override
    public void runOpMode() throws InterruptedException {
            delivery = new THEBOOT(hardwareMap);
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
                if(gamepad1.x){
                    delivery.setIn();
                    sleep(2000);
                }
                delivery.setPosition(DEFAULT_POSITION);
                telemetry.update();

            }
        }
    }

