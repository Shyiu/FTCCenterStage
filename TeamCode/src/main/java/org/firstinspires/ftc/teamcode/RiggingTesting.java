package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subclasses.Rigging;

public class RiggingTesting extends LinearOpMode {
    Rigging rigging;
    @Override
    public void runOpMode(){
        rigging = new Rigging(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){
            rigging.setRiggingPower(gamepad2.left_stick_y/-3.0);
            telemetry.addData("position", rigging.getRiggingPosition());
            telemetry.update();
        }
    }

}
