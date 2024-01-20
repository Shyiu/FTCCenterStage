package org.firstinspires.ftc.teamcode.PurePursuitTutorial.Main;



import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.ComputerDebugging;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.FloatPoint;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.com.company.Robot;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode.MyOpMode;
import org.firstinspires.ftc.teamcode.PurePursuitTutorial.treamcode.OpMode;

public class Main {
    public Main() {
    }

    public static void main(String[] args) {
        (new Main()).run();
    }

    public void run() {
        new ComputerDebugging();
        Robot robot = new Robot();
        OpMode opMode = new MyOpMode();
        opMode.init();
        ComputerDebugging.clearLogPoints();
        long var4 = System.currentTimeMillis();

        try {
            Thread.sleep(1000L);
        } catch (InterruptedException var8) {
            var8.printStackTrace();
        }

        while(true) {
            opMode.loop();

            try {
                Thread.sleep(30L);
            } catch (InterruptedException var7) {
                var7.printStackTrace();
            }

            robot.update();
            ComputerDebugging.sendRobotLocation(robot);
            ComputerDebugging.sendLogPoint(new FloatPoint(Robot.worldXPosition, Robot.worldYPosition));
            ComputerDebugging.markEndOfUpdate();
        }
    }
}
