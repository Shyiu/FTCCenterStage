package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.Unicorn;

@TeleOp
@Config
public class AutoDeliverytest extends LinearOpMode {
    Unicorn delivery ;
    public static double position = 0;
    public static double increment = 0.05;
    public static boolean move_slowly = false;
    @Override
    public void runOpMode() throws InterruptedException {
            delivery = new Unicorn(hardwareMap, telemetry);
            delivery.init();
            waitForStart();
            while(!isStopRequested() && opModeIsActive()) {
                if(!move_slowly) {
                    delivery.goToPosition(position);
                }
                else{
                    delivery.move_slowly_to(position, increment);
                }
                delivery.telemetry();
                telemetry.update();
            }

        }
    }

