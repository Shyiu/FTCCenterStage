package org.firstinspires.ftc.teamcode.lib;

import org.checkerframework.checker.units.qual.min;

import java.util.HashMap;

public class NanoMap {
    private HashMap<Double, Double> map;
    private double max_key;
    private double min_key;
    private double max_value;
    private double min_value;
    public NanoMap(){
        max_key = Double.MIN_VALUE;
        min_key = Double.MAX_VALUE;
        map = new HashMap<>();
        max_value = 0;
        min_value = 0;
    }
    public void put(double key, double value){
        if(key < min_key){
            min_key = key;
            min_value = value;
        }
        if(key > max_key){
            max_key = key;
            max_value = value;
        }
        map.put(key, value);
    }
    public double get(double key){
        if(!map.containsKey(key)){
            double distance_to_max = Math.abs(key - max_key);
            double distance_to_min = Math.abs(key - min_key);
            if(distance_to_max < distance_to_min){
                return max_value;
            }
            return min_value;
        }
        return map.get(key);
    }
}
