package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class MRDistance extends Subsystem{

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private MecanumBotConstant mc;
    private double offset = 6;
    Telemetry telemetry;
    public MRDistance(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, mc.distance_sensor);
    }
    public double getDist() {
        return rangeSensor.getDistance(DistanceUnit.INCH);
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
