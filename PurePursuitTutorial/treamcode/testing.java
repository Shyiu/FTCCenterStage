package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;

import org.firstinspires.ftc.teamcode.PurePursuitTutorial.org.opencv.core.Point;

public class testing {
    public static void main(String[] args){
        LineSegment segment = new LineSegment(0,0,3,3);
        System.out.println(segment.distToPoint(new Point(4,4)));

    }
}
