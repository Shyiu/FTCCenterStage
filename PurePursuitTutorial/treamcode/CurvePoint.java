package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;


import org.firstinspires.ftc.teamcode.PurePursuitTutorial.org.opencv.core.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnRadians, double slowDownTurnAmount) {
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint thisPoint) {
        this.x = thisPoint.x;
        this.y = thisPoint.y;
        this.moveSpeed = thisPoint.moveSpeed;
        this.turnSpeed = thisPoint.turnSpeed;
        this.followDistance = thisPoint.followDistance;
        this.slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        this.slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        this.pointLength = thisPoint.pointLength;
    }

    public Point toPoint() {
        return new Point(this.x, this.y);
    }

    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }
}
