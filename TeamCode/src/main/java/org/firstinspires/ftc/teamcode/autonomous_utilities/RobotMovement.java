package org.firstinspires.ftc.teamcode.autonomous_utilities;



import static org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot.worldAngle_rad;
import static org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot.worldXPosition;
import static org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot.worldYPosition;

import java.util.ArrayList;


import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.MovementVars;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Range;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;

public class RobotMovement {
    private static int end_index = 1;
    private static int slice = 0;
    private static ArrayList<Integer> visited = new ArrayList<>();
    public static double initialFollow = 50;
    public static double absoluteAngleToTarget;
    public RobotMovement() {
    }


    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {
        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90.0D)));
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double abs_distance = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);
        double movementXPower = relativeXToPoint / abs_distance;
        double movementYPower = relativeYToPoint / abs_distance;

        MovementVars.movement_x = movementXPower * movementSpeed;
        MovementVars.movement_y = movementYPower * movementSpeed;
        MovementVars.expected_velocity = Math.sqrt(MovementVars.expected_velocity * MovementVars.expected_velocity + 2 * DriveConstants.MAX_ACCEL * distanceToTarget);
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180.0) + preferredAngle;
        MovementVars.movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30.0), -1.0D, 1.0D) * turnSpeed;
    }
}
