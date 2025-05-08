package org.firstinspires.ftc.teamcode.autonomous_utilities;


public class LineSegment {

    private double m;
    private double x1;
    private double y1;
    private double x2;
    private double y2;

    public LineSegment(CurvePoint p1, CurvePoint p2){
        double changeX = p1.x - p2.x;
        changeX = Math.max(0.003, changeX);
        m = (p1.y - p2.y)/(changeX);
        x1 = p1.x;
        y1 = p1.y;
        x2 = p2.x;
        y2 = p2.y;
    }
    public LineSegment(double x1, double y1, double x2, double y2){
        double changeX = x1 -x2;
        double changeY = y1 -y2;
        changeX = Math.max(0.003, changeX);
        changeY = Math.max(0.003, changeY);
        m = (changeY/changeX);
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
    }
    public LineSegment[] split(double percentage){
        double splitX = x1 + percentage * (x2-x1);
        double splitY = y1 + percentage * (y2 - y1);
        return new LineSegment[]{new LineSegment(x1,y1, splitX, splitY), new LineSegment(splitX, splitY, x2, y2)};
    }
    public LineSegment(LineSegment l){
        x1 = l.x1;
        x2 = l.x2;
        y1 = l.y1;
        y2 = l.y2;
    }
    public static CurvePoint intersect(CurvePoint startLine, CurvePoint endLine, Point target){
        double changeX = startLine.x - endLine.x;
        changeX = Math.max(0.003, changeX);
        double m = (startLine.y - endLine.y)/(changeX);
        double x1 = startLine.x;
        double y1 = startLine.y;
        double x2 = endLine.x;
        double y2 = endLine.y;
        double m2 = -1/m;
        try{

            double intersectX = (y1 -y2 + m2 * x2 - m*x1)/(m2-m);
            double intersectY = m* (intersectX - x1) + y1;
            CurvePoint output = new CurvePoint(startLine);
            output.x = intersectX;
            output.y = intersectY;
            if (intersectX >= x1 && intersectX <= x2 && intersectY >= y1 && intersectY <= y2){
                return output;
            }
            else{
                return startLine;
            }
        }
        catch(Exception e){
        }
        return startLine;
    }

    public double distToPoint(Point target){
        double x2 = target.x;
        double y2 = target.y;
        double m2 = -1/m;
        try{

            double intersectX = (y1 -y2 + m2 * x2 - m*x1)/(m2-m);
            double intersectY = m* (intersectX - x1) + y1;

            if (intersectX >= x1 && intersectX <= this.x2 && intersectY >= y1 && intersectY <= this.y2){
                return Math.hypot(x2 - intersectX, y2-intersectY);
            }
            else{
                double distanceToEndPoint1 = Math.hypot(x2 - x1, y2-y1);
                double distanceToEndPoint2 = Math.hypot(x2 - this.x2, y2-this.y2);
                return Math.min(distanceToEndPoint1, distanceToEndPoint2);
            }

        }
        catch(Exception e){
        }
        return 0;
    }
}
