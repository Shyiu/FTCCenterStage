package org.firstinspires.ftc.teamcode.PurePursuitTutorial.org.opencv.core;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point() {
        this(0.0D, 0.0D);
    }

    public Point(double[] vals) {
        this();
        this.set(vals);
    }

    public void set(double[] vals) {
        if (vals != null) {
            this.x = vals.length > 0 ? vals[0] : 0.0D;
            this.y = vals.length > 1 ? vals[1] : 0.0D;
        } else {
            this.x = 0.0D;
            this.y = 0.0D;
        }

    }

    public Point clone() {
        return new Point(this.x, this.y);
    }

    public double dot(Point p) {
        return this.x * p.x + this.y * p.y;
    }

    public int hashCode() {
        int result = 1;
        long temp = Double.doubleToLongBits(this.x);
        result = 31 * result + (int)(temp ^ temp >>> 32);
        temp = Double.doubleToLongBits(this.y);
        result = 31 * result + (int)(temp ^ temp >>> 32);
        return result;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        } else if (!(obj instanceof Point)) {
            return false;
        } else {
            Point it = (Point)obj;
            return this.x == it.x && this.y == it.y;
        }
    }

    public String toString() {
        return "{" + this.x + ", " + this.y + "}";
    }
}
