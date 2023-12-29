package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.RiggingOld;

@TeleOp(name = "OldRiggingTest")
public class OldRiggingTesting extends LinearOpMode {
    RiggingOld rigging;
    @Override
    public void runOpMode(){
        rigging = new RiggingOld(hardwareMap, telemetry);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            rigging.setMotorLeftPower(-gamepad2.left_stick_y);
            rigging.setMotorRightPower(-gamepad2.right_stick_y);
            rigging.telemetry();
            telemetry.update();
        }
    }

}
