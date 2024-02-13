package org.firstinspires.ftc.teamcode.autonomous_utilities;


import java.util.ArrayList;
import java.util.Vector;


public class MathFunctions {
    public MathFunctions() {
    }

    public static double AngleWrap(double angle) {
        while (angle < -3.141592653589793D) {
            angle += 6.283185307179586D;
        }

        while (angle > 3.141592653589793D) {
            angle -= 6.283185307179586D;
        }

        return angle;
    }
    public static double AngleWrapPos(double angle){
        while (angle < 0) {
            angle += 6.283185307179586D;
        }

        while (angle > 2*Math.PI) {
            angle -= 6.283185307179586D;
        }
        return angle;
    }
    public static double Dot(Vector<Double> v1, Vector<Double> v2){
        return v1.get(0) * v2.get(0) + v1.get(1) * v2.get(1);
    }
    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        double dy1 = linePoint1.y, dx1 = linePoint1.x, dy2 = linePoint2.y, dx2 = linePoint2.x;
        double dy = dy1 - dy2, dx = dx1 - dx2;
        dy += dy == 0 ? 0.003 : 0;
        dx += dx == 0 ? 0.003 : 0;
        Vector<Double> d = new Vector<Double>();
        d.add(dx);
        d.add(dy);
        Vector<Double> f = new Vector<Double>();
        f.add(linePoint1.x - circleCenter.x);
        f.add(linePoint1.y - circleCenter.y);
        double a = Dot(d,d);
        double b = 2 * Dot(f,d);
        double c = Dot(f,f) - radius*radius;
        double discriminant = b*b - 4*a*c;
        double t1, t2;
        ArrayList allPoints = new ArrayList();

        if(discriminant < 0){
            return allPoints;
        }else{
            discriminant = Math.sqrt(discriminant);
            t1 = (-b - discriminant)/(2*a);
            t2 = (-b + discriminant)/(2*a);
            if(t1 >= 0 && t1 <= 1){
                Point p1 = new Point(linePoint1.x + t1 * d.get(0), linePoint1.y + t1 * d.get(1));
                allPoints.add(p1);
                return allPoints;
            }
            if(t2 >= 0 && t2 <= 1){
                Point p2 = new Point(linePoint1.x + t2 * d.get(0), linePoint1.y + t2 * d.get(1));
                allPoints.add(p2);
                return allPoints;
            }
        }
        return allPoints;
    }


}

