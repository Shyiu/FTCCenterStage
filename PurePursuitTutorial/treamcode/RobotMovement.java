package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;


import static org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot.worldYPosition;

import android.sax.EndElementListener;

import org.firstinspires.ftc.teamcode.PurePursuitTutorial.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Range;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot;

import java.util.ArrayList;


import org.firstinspires.ftc.teamcode.PurePursuitTutorial.org.opencv.core.Point;

public class RobotMovement {
    private static int end_index = 1;
    private static int slice = 0;
    private static ArrayList<Integer> visited = new ArrayList<>();
    public static double initialFollow = 50;
    public RobotMovement() {
    }

    //checks to make sure that the curve is processed correctly.
    public static void handleCurve(ArrayList<CurvePoint> allPoints){
        if (allPoints.get(allPoints.size() - 1).endpoint == false){
            CurvePoint startLine = allPoints.get(allPoints.size() - 2);
            CurvePoint endLine = allPoints.get(allPoints.size() - 1);

            //Adding a intermediate point on the last line to allow the robot to effectively stop.
            allPoints.add(allPoints.size() -1 , MathFunctions.compositeLine(startLine, endLine, .85));
        }
        if(visited.size() == 0){
            visited.add(0);
        }
//        System.out.println(visited.toString());
        followCurve(allPoints);

    }

    //Telling the robot to move towards the next point on the curve.
    public static void followCurve(ArrayList<CurvePoint> allPoints){
        for(int i = 0 ; i < allPoints.size() - 1; i++) {
            ComputerDebugging.sendLine(new FloatPoint( allPoints.get(i).x, allPoints.get(i).y), new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
        }


        CurvePoint followMe = getFollowPointPath(allPoints, new Point(Robot.worldXPosition, Robot.worldYPosition));
        System.out.println(followMe.toString());
        System.out.println(end_index);
        System.out.println(slice);
        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));
        if(end_index != allPoints.size() - 1) {
            goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followMe.endAngle, followMe.turnSpeed);
        }
        else{
            followMe = new CurvePoint(allPoints.get(allPoints.size() - 1));
            goToLastPosition(followMe.x, followMe.y, followMe.moveSpeed, followMe.endAngle, followMe.turnSpeed);
        }

    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation) {
//        slice = visited.get(visited.size() - 1);

        if (findSlice(robotLocation, pathPoints) - slice > 1){
            slice = Math.max(visited.get(visited.size() - 1), slice+1);
        }
        else if (findSlice(robotLocation, pathPoints) - slice == 1){
            slice += 1;
        }

        CurvePoint followMe = new CurvePoint(pathPoints.get(slice));
        for(int i = slice; i < pathPoints.size() - 1; i++){
           double followRadius = initialFollow;
           CurvePoint startLine = pathPoints.get(i);
           CurvePoint endLine = pathPoints.get(i + 1);
           CurvePoint robotPoint = new CurvePoint(LineSegment.intersect(startLine, endLine, robotLocation));

           ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

           double closestAngle = Integer.MAX_VALUE;

           for (Point thisIntersection: intersections){

                   if ((end_index != slice) || (thisIntersection.x >= robotPoint.x && thisIntersection.y >= robotPoint.y && thisIntersection.x <= endLine.x && thisIntersection.y <= endLine.y)) {
                       double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
                       double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                       if (deltaAngle < closestAngle) {
                           followMe.setPoint(thisIntersection);
                           closestAngle = deltaAngle;
                           end_index = i + 1;
                           visited.add(i);
                       }
                   }



           }
        }
        return followMe;
    }
    private static int findSlice(Point position, ArrayList<CurvePoint> allPoints){
        double minDist = 100000;
        int minDistSlice = 0;
        LineSegment closestSegment =  new LineSegment(0,0,0,0);
        for (int i = 0; i < allPoints.size() -1;i++){
            LineSegment segment = new LineSegment(allPoints.get(i), allPoints.get(i+1));
            if (segment.distToPoint(position) < minDist){
                minDist = segment.distToPoint(position);
                minDistSlice = i;
                closestSegment = new LineSegment(segment);


            }

        }
        LineSegment[] split = closestSegment.split(.80);

        if (split[0].distToPoint(position) > split[1].distToPoint(position)){
            minDistSlice += 1;
        }
        return minDistSlice;


    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90.0D)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));



        MovementVars.movement_x = movementXPower * movementSpeed;
        MovementVars.movement_y = movementYPower * movementSpeed;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        MovementVars.movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30.0), -1.0D, 1.0D) * turnSpeed;



    }
    public static void goToLastPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90.0D)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double reductionX = .006 * relativeXToPoint;
        double reductionY = .006 * relativeYToPoint;


        MovementVars.movement_x = movementXPower * movementSpeed * reductionX;
        MovementVars.movement_y = movementYPower * movementSpeed * reductionY;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        MovementVars.movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30.0), -1.0D, 1.0D) * turnSpeed;

        if (Math.abs(distanceToTarget) <= 10) {
            MovementVars.movement_turn = 0.0;
        }

    }
}
