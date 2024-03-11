package org.firstinspires.ftc.teamcode.lib;

import java.util.HashMap;

public class BucketEulersApproximation {

    private int steps;
    private double max_value = 0.45;
    private double min_value = 0.26;
    private double min_distance = 7.3;
    private double max_distance = 1.8;
    private NanoMap map;
    private NanoMap distance;
    private NanoMap reverse_distance;
    public BucketEulersApproximation(int steps, boolean old_delivery){
        this.steps = steps;

        map = new NanoMap();

        distance = new NanoMap();

        reverse_distance = new NanoMap();



        //Specific Data for the Arm
        map.put(2200.0,0.1);
        map.put(2250.0,0.05);
        map.put(2300.0,0.03);
        map.put(2350.0,0.03);
        map.put(2400.0,0.06);
        map.put(2450.0,0.08);
        map.put(2500.0,0.1);
        map.put(2550.0,0.12);

        distance.put(2200.0, 1.8);
        distance.put(2250.0, 1.8);
        distance.put(2300.0, 2.2);
        distance.put(2350.0, 2.5);
        distance.put(2400.0, 4.6);
        distance.put(2450.0, 5.9);
        distance.put(2500.0, 6.6);
        distance.put(2550.0, 7.3);

        reverse_distance.put(3.0, 2381);
        reverse_distance.put(4.0, 2434);
        reverse_distance.put(5.0, 2541);
        reverse_distance.put(6.0, 2550);

    }

    public BucketEulersApproximation(int steps){
        this.steps = steps;

        map = new NanoMap();

        distance = new NanoMap();

        reverse_distance = new NanoMap();

        //Specific Data for the Arm

        map.put(1800.0, 0.26);
        map.put(1850.0, 0.27);
        map.put(1900.0, 0.27);
        map.put(1950.0, 0.3);
        map.put(2000.0, 0.28);
        map.put(2050.0, 0.3);
        map.put(2100.0, 0.32);
        map.put(2150.0, 0.32);
        map.put(2200.0,0.38);
        map.put(2250.0,0.38);
        map.put(2300.0,0.38);
        map.put(2350.0,0.4);
        map.put(2400.0,0.46);
        map.put(2450.0,0.4);
        map.put(2500.0,0.45);


        distance.put(1800.0, 0.0);
        distance.put(1850.0, 0.0);
        distance.put(1900.0, 0.0);
        distance.put(1950.0, 1.8);
        distance.put(2000.0, 2.4);
        distance.put(2050.0, 4.0);
        distance.put(2100.0, 5.0);
        distance.put(2150.0, 6.0);
        distance.put(2200.0, 7.0);
        distance.put(2250.0, 8.0);
        distance.put(2300.0, 9.0);
        distance.put(2350.0, 10.3);
        distance.put(2400.0, 11.2);
        distance.put(2450.0, 12.5);
        distance.put(2500.0, 13.0);
        distance.put(2550.0, 13.6);

        reverse_distance.put(2.0, 1950.0);
        reverse_distance.put(3.0, 2000);
        reverse_distance.put(4.0, 2066);
        reverse_distance.put(5.0, 2136);
        reverse_distance.put(6.0, 2154);
        reverse_distance.put(7.0, 2195);
        reverse_distance.put(8.0, 2228);
        reverse_distance.put(9.0, 2270);
        reverse_distance.put(10.0, 2328);
        reverse_distance.put(11.0, 2371);
        reverse_distance.put(12.0, 2415);
        reverse_distance.put(13.0, 2506);
        reverse_distance.put(14.0, 2550);






    }
    public double getDistanceApproximation(double value){
        return getApproximationForHashMap(distance, value);
    }
    public int getArmLimit(double distance){
        return (int) getApproximationForHashMap(reverse_distance, distance, 1);
    }

    public double getApproximationForHashMap(NanoMap map, double value){
        int low_value = steps * (int) Math.floor(value/steps);
        int high_value = steps * (int) Math.ceil(value/steps);
        double slope = (map.get(high_value) - map.get(low_value) )/(steps);
        double y = slope * (value - low_value) + map.get(low_value);
        return y;
    }
    public double getApproximationForHashMap(NanoMap map, double value, int steps){
        int low_value = steps * (int) Math.floor(value/steps);
        int high_value = steps * (int) Math.ceil(value/steps);
        double slope = (map.get(high_value) - map.get(low_value) )/(steps);
        double y = slope * (value - low_value) + map.get(low_value);
        return y;
    }

    public double getApproximation(double value){
        return getApproximationForHashMap(map, value);
    }
}
