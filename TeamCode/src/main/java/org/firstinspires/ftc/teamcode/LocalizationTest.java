package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

@Config
@TeleOp
public class LocalizationTest extends LinearOpMode {
    MecanumBotConstant m = new MecanumBotConstant();
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    private FtcDashboard dashboard;


    Encoder linear;
    Encoder lateral;
    Robot r;

    @Override
    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        fr = hardwareMap.get(DcMotor.class, m.fr);
        fl = hardwareMap.get(DcMotor.class, m.fl);
        br = hardwareMap.get(DcMotor.class, m.br);
        bl = hardwareMap.get(DcMotor.class, m.bl);

        lateral = new Encoder(hardwareMap, fr);
        linear = new Encoder(hardwareMap, fl);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();


        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        r = new Robot(hardwareMap, fr, fl, br, bl, lateral, linear);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            double leftpower=  sameSignSqrt(-gamepad1.left_stick_y/2);
            double rightpower =  sameSignSqrt(-gamepad1.right_stick_y/2);
            r.setMotorPowers(leftpower, rightpower, leftpower, rightpower);

            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            drawRobot(fieldOverlay);

            r.updatePosition();
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("X", r.getCurrentPosition()[0]);
            telemetry.addData("Y", r.getCurrentPosition()[1]);
            telemetry.addData("LeftPower", leftpower);
            telemetry.addData("RightPower", rightpower);
            telemetry.update();
        }

    }
    public void drawRobot(Canvas canvas) {

    }
    public double sameSignSqrt(double input){
        return Math.copySign(Math.sqrt(Math.abs(input)), input);
    }
}
