package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.opengl.shaders.CubeMeshVertexShader;
import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;

@Config
@TeleOp(name = "ShivaniRiggingTest")
public class ShivaniRiggingTesting extends LinearOpMode {
    public static boolean retracted = true;
    public static double motor_power = 0;
    ShivaniRigging rigging;
    @Override
    public void runOpMode(){
        rigging = new ShivaniRigging(hardwareMap, telemetry);
        rigging.init();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if (retracted || gamepad1.x) {
                rigging.retract();
            }else{
                rigging.extend();
            }
            rigging.setRiggingPower(motor_power == 0 ? -gamepad1.left_stick_y : motor_power);
            rigging.telemetry();
            telemetry.update();
        }
    }

}
