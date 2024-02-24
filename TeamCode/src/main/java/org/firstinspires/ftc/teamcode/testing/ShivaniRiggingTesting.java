package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;

@Config
@TeleOp(name = "ShivaniRiggingTest")
public class ShivaniRiggingTesting extends LinearOpMode {

    public static boolean update = false;
    public static boolean release_hooks = false;

    public static double motor_power = 0;
    public static double hook_power = 0;
    public static int TARGET = 0;
    ShivaniRigging rigging;
    @Override
    public void runOpMode() throws InterruptedException {
        rigging = new ShivaniRigging(hardwareMap, telemetry);
        rigging.init();

        waitForStart();
        rigging.activate();

        while(!isStopRequested() && opModeIsActive()){

            rigging.setHookPower(hook_power == 0 ? -gamepad1.right_stick_y : hook_power);
            if (update){
                rigging.update();
                if(!rigging.isCompletedFor(1)){
                    rigging.release_motor();
                }
            }
            rigging.set_hook_target_position(TARGET);
            if(release_hooks){
                rigging.release_hooks();
                release_hooks = false;
            }
            rigging.setRiggingPower(motor_power == 0 ? -gamepad1.left_stick_y : motor_power);
            telemetry.addData("gamepad1", -gamepad1.left_stick_y);
            rigging.telemetry();
            telemetry.update();
        }
    }

}
