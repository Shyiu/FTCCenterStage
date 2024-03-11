package org.firstinspires.ftc.teamcode.subclasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

@Config
public class Distance extends Subsystem{
    private DistanceSensor sensorDistance;

    private MecanumBotConstant mc;
    private double offset = 1.5;
    private double past_distance_reading = 0;
    private double current_dist = 0;
    private double filtered_position = 0;
    public static double filter_value = 0;
    private double delay = 0;
    private ElapsedTime timer;
    private boolean right = false;
    Telemetry telemetry;
    public Distance(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        timer = new ElapsedTime();

        sensorDistance = hardwareMap.get(DistanceSensor.class, mc.front_distance);

    }
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, boolean right){
        this.right = right;
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        if(right) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, mc.right_distance);
        }else{
            sensorDistance = hardwareMap.get(DistanceSensor.class, mc.left_distance);
        }
        timer = new ElapsedTime();

        past_distance_reading = getDist();
        current_dist = getDist();
        filtered_position = getDist();
    }
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        this.timer = timer;

        sensorDistance = hardwareMap.get(DistanceSensor.class, mc.front_distance);

    }
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, boolean right, ElapsedTime timer){
        this.right = right;
        this.telemetry = telemetry;
        mc = new MecanumBotConstant();
        if(right) {
            sensorDistance = hardwareMap.get(DistanceSensor.class, mc.right_distance);
        }else{
            sensorDistance = hardwareMap.get(DistanceSensor.class, mc.left_distance);
        }        this.timer = timer;

        past_distance_reading = getDist();
        current_dist = getDist();
        filtered_position = getDist();
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
    public void setFilter(double filter){
        filter_value = filter;
    }
    @Override
    public void telemetry() {
        String label = right ? "Right " : "Left ";
        telemetry.addData(label + "Robot Edge Distance", getDistFromRobotEdge());
        telemetry.addData(label + "Distance (in)", getDist());
    }
    public void update(){
        if(timer.seconds() - delay > 0.005){
            past_distance_reading = current_dist;
            current_dist = getDist();
            if(Math.abs(past_distance_reading - current_dist) > 1){
                filtered_position = current_dist;
            }else {
                filtered_position = filter_value * current_dist + (1 - filter_value) * past_distance_reading;
            }
            delay = timer.seconds();
        }
    }
    public double getFilteredDist(){
        return filtered_position;
    }
    @Override
    public void init() {
        return;
    }
}
