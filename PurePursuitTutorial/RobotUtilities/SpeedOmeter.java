package org.firstinspires.ftc.teamcode.PurePursuitTutorial.RobotUtilities;


import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode.MathFunctions;


public class SpeedOmeter {
    private static long lastUpdateStartTime = 0L;
    private static double currSpeedY = 0.0D;
    private static double currSpeedX = 0.0D;
    public static int timeBetweenUpdates = 25;
    public static double yDistTraveled = 0.0D;
    public static double xDistTraveled = 0.0D;
    public static double lastAngle = 0.0D;
    public static double angularVelocity = 0.0D;
    public static double scalePrediction = 1.0D;
    public static double ySlipDistanceFor1CMPS;
    public static double xSlipDistanceFor1CMPS;
    public static double turnSlipAmountFor1RPS;

    public SpeedOmeter() {
    }

    public static void update() {
        long currTime = System.currentTimeMillis();
        if (!(Math.abs(yDistTraveled) < 1.0E-9D) || !(Math.abs(xDistTraveled) < 1.0E-9D) || !(Math.abs(angularVelocity) < 1.0E-6D)) {
            if (currTime - lastUpdateStartTime > (long)timeBetweenUpdates) {
                double elapsedTime = (double) (currTime - lastUpdateStartTime) / 1000.0D;
                double speedY = yDistTraveled / elapsedTime;
                double speedX = xDistTraveled / elapsedTime;
                if (speedY < 200.0D && speedX < 200.0D) {
                    currSpeedY = speedY;
                    currSpeedX = speedX;
                }

                angularVelocity = MathFunctions.AngleWrap(Robot.worldAngle_rad - lastAngle) / elapsedTime;
                lastAngle = Robot.worldAngle_rad;
                yDistTraveled = 0.0D;
                xDistTraveled = 0.0D;
                lastUpdateStartTime = currTime;
            }

        }
    }

    public static double getSpeedY() {
        return currSpeedY;
    }

    public static double getSpeedX() {
        return currSpeedX;
    }

    public static double getDegPerSecond() {
        return Math.toDegrees(angularVelocity);
    }

    public static double getRadPerSecond() {
        return angularVelocity;
    }

    public static double currSlipDistanceY() {
        return getSpeedY() * ySlipDistanceFor1CMPS;
    }

    public static double currSlipDistanceX() {
        return getSpeedX() * xSlipDistanceFor1CMPS;
    }

    public static double currSlipAngle() {
        return getRadPerSecond() * turnSlipAmountFor1RPS;
    }

    static {
        ySlipDistanceFor1CMPS = 0.14D * scalePrediction;
        xSlipDistanceFor1CMPS = 0.153D * scalePrediction;
        turnSlipAmountFor1RPS = 0.09D * scalePrediction;
    }
}
