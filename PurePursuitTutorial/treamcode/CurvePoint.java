package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;

import org.firstinspires.ftc.teamcode.PurePursuitTutorial.org.opencv.core.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;
    public double endAngle;
    public boolean endpoint = false;
    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance,  double slowDownTurnRadians, double slowDownTurnAmount, double endAngle){
        this.x = x;
        this.y = y;
        this.endAngle = endAngle;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.slowDownTurnRadians = slowDownTurnRadians;

    }public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance,  double slowDownTurnRadians, double endAngle, double slowDownTurnAmount, boolean end){
        this.x = x;
        this.y = y;
        this.endAngle = endAngle;

        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnAmount = slowDownTurnAmount;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.endpoint = true;

    }
    public CurvePoint(CurvePoint thisPoint){
        x = thisPoint.x;
        y = thisPoint.y;
        this.endAngle = thisPoint.endAngle;

        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;

    }
    public CurvePoint(CurvePoint thisPoint, boolean end){
        x = thisPoint.x;
        y = thisPoint.y;
        this.endAngle = thisPoint.endAngle;

        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        endpoint = end;

    }

    public CurvePoint() {

    }

    public Point toPoint(){
        return new Point(x,y);

    }

    public void setPoint(Point thisIntersection) {
        x = thisIntersection.x;
        y = thisIntersection.y;
    }
    public String toString(){
        return "(" + x + "," + y + ")";
    }
}

