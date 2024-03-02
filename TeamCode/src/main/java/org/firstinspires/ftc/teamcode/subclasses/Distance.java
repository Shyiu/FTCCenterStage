package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Distance extends Subsystem{
    private DistanceSensor sensorDistance;

    private MecanumBotConstant mc;
    private double offset = 1.5;
    private boolean front = false;
    Telemetry telemetry;
    public Distance(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        sensorDistance = hardwareMap.get(DistanceSensor.class, mc.distance_sensor);
    }
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, boolean front){
        this.front = front;
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        sensorDistance = hardwareMap.get(DistanceSensor.class, mc.front_distance);
    }
    public double getDist() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
    public double getDistFromRobotEdge(){
        return getDist() - offset;
    }
    public void setOffset(double offset){
        this.offset = offset;
    }
    @Override
    public void telemetry() {
        String label = front ? "Front " : "Back ";
        telemetry.addData(label + "Robot Edge Distance", getDistFromRobotEdge());
        telemetry.addData(label + "Distance (in)", getDist());
    }

    @Override
    public void init() {
        return;
    }
}
