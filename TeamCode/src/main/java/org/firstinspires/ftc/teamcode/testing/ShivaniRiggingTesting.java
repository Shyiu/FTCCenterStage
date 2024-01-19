package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;

@Config
@TeleOp(name = "ShivaniRiggingTest")
public class ShivaniRiggingTesting extends LinearOpMode {
    public static boolean retracted_right = true;
    public static boolean retracted_left = true;
    public static boolean left_lock = true;
    public static boolean right_lock = true;
    public static boolean raise_wheels = false;
    public static boolean raise_rigging_wheels = false;

    public static double motor_power = 0;

    ShivaniRigging rigging;
    @Override
    public void runOpMode(){
        rigging = new ShivaniRigging(hardwareMap, telemetry);
        rigging.init();

        waitForStart();
        rigging.activate();


        while(!isStopRequested() && opModeIsActive()){
            if (!left_lock){
                if (retracted_left || gamepad1.x) {
                    rigging.close_left();
                }else{
                    rigging.open_left();
                }
            }else{
                rigging.freeze_left();
            }
            if (!right_lock) {
                if (retracted_right || gamepad1.x) {
                    rigging.close_right();
                } else {
                    rigging.open_right();
                }
            }else{
                rigging.freeze_right();
            }
            if (raise_wheels){
                rigging.update();
            }
            if (raise_rigging_wheels){
                rigging.raise_rigging_motor();
            }


            rigging.setRiggingPower(motor_power == 0 ? -gamepad1.left_stick_y : motor_power);
            rigging.telemetry();
            telemetry.update();
        }
    }

}
