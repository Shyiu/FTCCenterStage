package org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company;


import org.firstinspires.ftc.teamcode.PurePursuitTutorial.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode.MathFunctions;

public class Robot {
    public static boolean usingComputer = true;
    public static double xSpeed = 0.0D;
    public static double ySpeed = 0.0D;
    public static double turnSpeed = 0.0D;
    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;
    private long lastUpdateTime = 0L;

    public Robot() {
        worldXPosition = 50.0D;
        worldYPosition = 50.0D;
        worldAngle_rad = Math.toRadians(-45.0D);
    }

    public double getXPos() {
        return worldXPosition;
    }

    public double getYPos() {
        return worldYPosition;
    }

    public double getWorldAngle_rad() {
        return worldAngle_rad;
    }

    public void update() {
        long currentTimeMillis = System.currentTimeMillis();
        double elapsedTime = (double)(currentTimeMillis - this.lastUpdateTime) / 1000.0D;
        this.lastUpdateTime = currentTimeMillis;
        if (!(elapsedTime > 1.0D)) {
            double totalSpeed = Math.hypot(xSpeed, ySpeed);
            double angle = Math.atan2(ySpeed, xSpeed) - Math.toRadians(90.0D);
            double outputAngle = worldAngle_rad + angle;
            worldXPosition += totalSpeed * Math.cos(outputAngle) * elapsedTime * 1000.0D * 0.2D;
            worldYPosition += totalSpeed * Math.sin(outputAngle) * elapsedTime * 1000.0D * 0.2D;
            worldAngle_rad += MovementVars.movement_turn * elapsedTime * 20.0D / 6.283185307179586D;
            worldAngle_rad = MathFunctions.AngleWrap(worldAngle_rad);
            xSpeed += Range.clip((MovementVars.movement_x - xSpeed) / 0.2D, -1.0D, 1.0D) * elapsedTime;
            ySpeed += Range.clip((MovementVars.movement_y - ySpeed) / 0.2D, -1.0D, 1.0D) * elapsedTime;
            turnSpeed += Range.clip((MovementVars.movement_turn - turnSpeed) / 0.2D, -1.0D, 1.0D) * elapsedTime;
            xSpeed *= 1.0D - elapsedTime;
            ySpeed *= 1.0D - elapsedTime;
            turnSpeed *= 1.0D - elapsedTime;
        }
    }
}
