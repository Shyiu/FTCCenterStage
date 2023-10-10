package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;


import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.PurePursuitTutorial.org.opencv.core.Point;

public class MathFunctions {
    public MathFunctions() {
    }

    public static double AngleWrap(double angle) {
        while(angle < -3.141592653589793D) {
            angle += 6.283185307179586D;
        }

        while(angle > 3.141592653589793D) {
            angle -= 6.283185307179586D;
        }

        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003D) {
            linePoint1.y = linePoint2.y + 0.003D;
        }

        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003D) {
            linePoint1.x = linePoint2.x + 0.003D;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double quadraticA = 1.0D + Math.pow(m1, 2.0D);
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;
        double quadraticB = 2.0D * m1 * y1 - 2.0D * Math.pow(m1, 2.0D) * x1;
        double quadraticC = Math.pow(m1, 2.0D) * Math.pow(x1, 2.0D) - 2.0D * y1 * m1 * x1 + Math.pow(y1, 2.0D) - Math.pow(radius, 2.0D);
        ArrayList allPoints = new ArrayList();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2.0D) - 4.0D * quadraticA * quadraticC)) / (2.0D * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2.0D) - 4.0D * quadraticA * quadraticC)) / (2.0D * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception var30) {
        }

        return allPoints;
    }
}
