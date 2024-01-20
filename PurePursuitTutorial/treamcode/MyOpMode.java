package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;


import java.util.ArrayList;

public class MyOpMode extends OpMode {

    @Override
    public void init(){

    }
    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(125.0,50.0,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
        allPoints.add(new CurvePoint(125.0,100,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
        allPoints.add(new CurvePoint(250,100,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
        allPoints.add(new CurvePoint(250,150,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
        allPoints.add(new CurvePoint(300,200,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
        allPoints.add(new CurvePoint(250,201,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));
        allPoints.add(new CurvePoint(100,201,1.0,1.0,50.0, Math.toRadians(50), 1.0, Math.toRadians(90.0)));

        RobotMovement.handleCurve(allPoints);
//        RobotMovement.goToPosition(250,100,20,Math.toRadians(90), Math.toRadians(90));


    }
}
