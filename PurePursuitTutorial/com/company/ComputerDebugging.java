package org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company;

import java.text.DecimalFormat;

public class ComputerDebugging {
    private static UdpServer udpServer;
    private static StringBuilder messageBuilder = new StringBuilder();
    private static DecimalFormat df = new DecimalFormat("#.00");

    public ComputerDebugging() {
        UdpServer.kill = false;
        udpServer = new UdpServer(11115);
        Thread runner = new Thread(udpServer);
        runner.start();
    }

    public static void sendRobotLocation(Robot robot) {
        if (Robot.usingComputer) {
            messageBuilder.append("ROBOT,");
            messageBuilder.append(df.format(robot.getXPos()));
            messageBuilder.append(",");
            messageBuilder.append(df.format(robot.getYPos()));
            messageBuilder.append(",");
            messageBuilder.append(df.format(robot.getWorldAngle_rad()));
            messageBuilder.append("%");
        }
    }

    public static void sendKeyPoint(FloatPoint floatPoint) {
        if (Robot.usingComputer) {
            messageBuilder.append("P,").append(df.format(floatPoint.x)).append(",").append(df.format(floatPoint.y)).append("%");
        }
    }

    public static void sendLogPoint(FloatPoint floatPoint) {
        if (Robot.usingComputer) {
            messageBuilder.append("LP,").append(df.format(floatPoint.x)).append(",").append(df.format(floatPoint.y)).append("%");
        }
    }

    public static void sendLine(FloatPoint point1, FloatPoint point2) {
        if (Robot.usingComputer) {
            messageBuilder.append("LINE,").append(df.format(point1.x)).append(",").append(df.format(point1.y)).append(",").append(df.format(point2.x)).append(",").append(df.format(point2.y)).append("%");
        }
    }

    public static void stopAll() {
        if (Robot.usingComputer) {
            UdpServer.kill = true;
        }
    }

    public static void markEndOfUpdate() {
        if (Robot.usingComputer) {
            messageBuilder.append("CLEAR,%");
            udpServer.splitAndSend(messageBuilder.toString());
            messageBuilder = new StringBuilder();
        }
    }

    public static void clearLogPoints() {
        if (Robot.usingComputer) {
            udpServer.splitAndSend("CLEARLOG,%");
        }
    }
}