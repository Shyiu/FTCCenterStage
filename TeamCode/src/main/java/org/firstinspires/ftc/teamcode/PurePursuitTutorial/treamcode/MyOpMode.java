package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;

public class MyOpMode extends OpMode{
    @Override
    public void init(){

    }

    @Override
    public void loop(){
        RobotMovement.goToPosition(358/2, 358/2,1.0, Math.toRadians(90), 0.3);


    }
}
