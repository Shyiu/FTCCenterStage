package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subclasses.Rigging;

@TeleOp(name = "RiggingTest")
public class RiggingTesting extends LinearOpMode {
    Rigging rigging;
    @Override
    public void runOpMode(){
        rigging = new Rigging(hardwareMap);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            rigging.setRiggingPower(-gamepad2.left_stick_y);
            telemetry.addData("position", rigging.getRiggingPosition());
            telemetry.addData("right", rigging.getRiggingRight());
            telemetry.addData("left", rigging.getRiggingLeft());
            telemetry.update();
        }
    }

}
