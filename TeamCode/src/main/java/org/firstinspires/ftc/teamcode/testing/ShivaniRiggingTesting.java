package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;

@Config
@TeleOp(name = "ShivaniRiggingTest")
public class ShivaniRiggingTesting extends LinearOpMode {

    public static boolean raise_hooks = false;
    public static boolean release_hooks = false;
    public static boolean update = false;
    public static boolean moving = false;

    public static double motor_power = 0;
    public static double hook_power = 0;
    public static double DAMPER_POSITION = 0.5;
    public static int TARGET = 0;
    ShivaniRigging rigging;
    @Override
    public void runOpMode() throws InterruptedException {
        rigging = new ShivaniRigging(hardwareMap, telemetry);
        rigging.init();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            rigging.moveDamper(DAMPER_POSITION);
            if (raise_hooks){
                rigging.raise_hooks_to_sensor();
                raise_hooks = false;
            }
            if(release_hooks){
                rigging.release_hooks();
                release_hooks = false;
            }
            telemetry.addData("Moving To", moving);
            telemetry.addData("Rigging Is Busy", rigging.isBusy());

            if(rigging.doneMoving() && moving){
                rigging.moveHook(TARGET);
                telemetry.addData("Moving To", TARGET);
                telemetry.update();
            }
            if(update) {
                rigging.update();
            }else {
                rigging.setRiggingPower(motor_power == 0 ? -gamepad1.left_stick_y : motor_power);
                rigging.setHookPower(hook_power == 0 ? -gamepad1.right_stick_y : hook_power);

            }
//
//
            telemetry.addData("gamepad1", -gamepad1.left_stick_y);
            rigging.telemetry();
            telemetry.update();
        }
    }

}
