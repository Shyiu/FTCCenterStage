package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class Color extends Subsystem{
    private NormalizedColorSensor colorSensor;

    private MecanumBotConstant mc;
    private double offset = 1.5;
    final float[] hsvValues = new float[3];

    Telemetry telemetry;
    public Color(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, mc.color_sensor);
        colorSensor.setGain(1);

    }
    public double getDist() {
        return ((DistanceSensor)colorSensor).getDistance(DistanceUnit.CM);
    }
    public double getDistFromRobotEdge(){
        return getDist() - offset;
    }
    public float getSaturation(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        android.graphics.Color.colorToHSV(colors.toColor(), hsvValues);

        return hsvValues[1];
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
