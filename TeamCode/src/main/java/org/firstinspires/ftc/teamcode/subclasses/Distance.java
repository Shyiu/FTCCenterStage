package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Distance extends Subsystem{
    private DistanceSensor sensorDistance;
    private MecanumBotConstant mc;
    private double offset = 6;
    Telemetry telemetry;
    public Distance(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        sensorDistance = hardwareMap.get(DistanceSensor.class, mc.distance);
    }
    public double getDist() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
    public double getDistFromRobotEdge(){
        return getDist() - offset;
    }

    @Override
    public void telemetry() {
        telemetry.addData("Robot Edge Distance", getDistFromRobotEdge());
        telemetry.addData("Distance (in)", getDist());
    }

    @Override
    public void init() {
        return;
    }
}
