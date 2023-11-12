package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.Lift;

@Config
@TeleOp(name = "servo_testing")
public class IntakeTest extends LinearOpMode {

    protected Servo turn1;
    protected Servo turn2;
    protected DcMotor arm;
    protected MecanumBotConstant m;
    public static double SERVO_POSITION = .5;
    public static double SERVO_POSITION_2 = .5;
    private static double SERVO_TWO_OFFSET = 0.03;
    private static double SERVO_ONE_OFFSET = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m = new MecanumBotConstant();

        turn1 = hardwareMap.get(Servo.class, m.servo1);
        turn2 = hardwareMap.get(Servo.class, m.servo2);
        turn1.setDirection(Servo.Direction.REVERSE);
        arm = hardwareMap.get(DcMotor.class, m.intake);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            if (gamepad2.a) {
                arm.setPower(.5);
            }
            else{
                arm.setPower(0);
            }
            turn1.setPosition(SERVO_POSITION);
            turn2.setPosition(SERVO_POSITION_2);
            telemetry.addData("Right Stick Y", gamepad2.right_stick_y *-1);
            telemetry.update();
        }

    }
    public double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
}
