package org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode;

import static org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot.*;

import com.qualcomm.robotcore.util.Range;

public class RobotMovement {
    public static void  goToPosition(double x, double y, double movement_speed, double preferredAngle, double turn_speed){
        double distanceToTarget = Math.hypot(y-worldYPosition, x-worldXPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition, x-worldXPosition);
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        xSpeed = movementXPower;
        ySpeed = movementYPower;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) - preferredAngle;
        turnSpeed = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turn_speed;

        if (distanceToTarget < 10){
            turnSpeed = 0;
        }

    }
}
