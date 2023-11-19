package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Distance {
    private DistanceSensor sensorDistance;
    private MecanumBotConstant mc;
    private double offset = 6;

    public Distance(HardwareMap hardwareMap){
        mc = new MecanumBotConstant();
        sensorDistance = hardwareMap.get(DistanceSensor.class, mc.distance);
    }
    public double getDist() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
    public double getDistFromRobotEdge(){
        return getDist() - offset;
    }

}
