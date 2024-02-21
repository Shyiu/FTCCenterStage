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
    public static double right_power = 0;
    public static double left_power = 0;
    ShivaniRigging rigging;
    @Override
    public void runOpMode() throws InterruptedException {
        rigging = new ShivaniRigging(hardwareMap, telemetry);
        rigging.init();

        waitForStart();
        rigging.activate();

        while(!isStopRequested() && opModeIsActive()){
            rigging.set_right_power(right_power == 0 ? -gamepad2.right_stick_y : right_power);
            rigging.set_left_power(left_power == 0 ? -gamepad2.left_stick_y : left_power);
            if(release_hooks){
                rigging.raise_hooks();
                release_hooks = false;
            }
            if (update){
                rigging.update();
            }
            rigging.setRiggingPower(motor_power == 0 ? -gamepad1.left_stick_y : motor_power);
            telemetry.addData("gamepad1", -gamepad1.left_stick_y);
            rigging.telemetry();
            telemetry.update();
        }
    }

}
