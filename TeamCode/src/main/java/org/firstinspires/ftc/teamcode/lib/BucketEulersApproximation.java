package org.firstinspires.ftc.teamcode.lib;

import java.util.HashMap;

public class BucketEulersApproximation {

    private int steps;
    private double max_value = 0.45;
    private double min_value = 0.26;
    private HashMap<Integer, Double> map;
    private HashMap<Integer, Double> distance;
    public BucketEulersApproximation(int steps, boolean old_delivery){
        this.steps = steps;

        map = new HashMap<>();
        max_value = 0.13;
        min_value = 0.02;

        //Specific Data for the Arm
        map.put(2200,0.02);
        map.put(2250,0.05);
        map.put(2300,0.08);
        map.put(2350,0.08);
        map.put(2400,0.08);
        map.put(2450,0.09);
        map.put(2500,0.121);

        distance.put(1800, 1.0);
        distance.put(1850, 1.0);
        distance.put(1900, 1.0);
        distance.put(1950, 1.0);
        distance.put(2000, 1.0);
        distance.put(2050, 1.0);
        distance.put(2100, 1.0);
        distance.put(2150, 1.0);
        distance.put(2200, 1.0);
        distance.put(2250, 1.0);
        distance.put(2300, 1.0);
        distance.put(2350, 1.0);
        distance.put(2400, 1.0);
        distance.put(2450, 1.0);
        distance.put(2500, 1.0);
        distance.put(2550, 1.0);
    }

    public BucketEulersApproximation(int steps){
        this.steps = steps;

        map = new HashMap<>();

        distance = new HashMap<>();

        //Specific Data for the Arm

        map.put(1800, 0.26);
        map.put(1850, 0.27);
        map.put(1900, 0.27);
        map.put(1950, 0.3);
        map.put(2000, 0.28);
        map.put(2050, 0.3);
        map.put(2100, 0.32);
        map.put(2150, 0.32);
        map.put(2200,0.38);
        map.put(2250,0.38);
        map.put(2300,0.38);
        map.put(2350,0.4);
        map.put(2400,0.46);
        map.put(2450,0.4);
        map.put(2500,0.45);


        distance.put(1800, 0.0);
        distance.put(1850, 0.0);
        distance.put(1900, 0.0);
        distance.put(1950, 1.8);
        distance.put(2000, 2.4);
        distance.put(2050, 4.0);
        distance.put(2100, 5.0);
        distance.put(2150, 6.0);
        distance.put(2200, 7.0);
        distance.put(2250, 8.0);
        distance.put(2300, 9.0);
        distance.put(2350, 10.3);
        distance.put(2400, 11.2);
        distance.put(2450, 12.5);
        distance.put(2500, 13.0);
        distance.put(2550, 13.6);




    }
    public double getDistanceApproximation(double value){
        int low_value = steps * (int) Math.floor(value/steps);
        int high_value = steps * (int) Math.ceil(value/steps);
        if(!distance.containsKey(low_value)){
            return min_value;
        }
        if(!distance.containsKey(high_value)){
            return max_value;
        }
        double slope = (distance.get(high_value) - distance.get(low_value) )/(steps);
        double y = slope * (value - low_value) + distance.get(low_value);
        return y;
    }
    public int getArmLimit(double distance){
        return 2600;
    }
    public double getApproximation(double value){
        int low_value = steps * (int) Math.floor(value/steps);
        int high_value = steps * (int) Math.ceil(value/steps);
        if(!map.containsKey(low_value)){
            return min_value;
        }
        if(!map.containsKey(high_value)){
            return max_value;
        }
        double slope = (map.get(high_value) - map.get(low_value) )/(steps);
        double y = slope * (value - low_value) + map.get(low_value);
        return y;
    }
}
