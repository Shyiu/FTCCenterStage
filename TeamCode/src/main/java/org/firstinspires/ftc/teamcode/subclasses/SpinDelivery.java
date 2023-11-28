package org.firstinspires.ftc.teamcode.subclasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;

public class SpinDelivery extends Subsystem{

    public static double pixel_1_collection = 1;
    public static double pixel_2_collection = .7;
    public static double pixel_1_drop = .5;
    public static double pixel_2_drop = .2;
    public enum POSITION{
        COLLECTION_1,
        COLLECTION_2,
        PIXEL_1,
        PIXEL_2,
        WAIT
    }
    private int index = 0;
    ElapsedTime delay;
    Servo servo;
    Telemetry telemetry;
    MecanumBotConstant mc;
    private POSITION pos;
    public SpinDelivery(HardwareMap hardwareMap, Telemetry telemetry){
        delay = new ElapsedTime();
        mc = new MecanumBotConstant();
        this.telemetry = telemetry;
        servo  = hardwareMap.get(Servo.class, mc.servo2);

    }

    @Override
    public void telemetry() {
        telemetry.addData("Delivery Position", pos);
    }

    @Override
    public void init() {
        pos = POSITION.COLLECTION_1;
        servo.setPosition(pixel_1_collection);
    }
    public void nextStage(){
        if(delay.time() > .6) {
            index += 1;
            index = index >= 4 ? 0 : index;
            switch(index) {
                case 0:
                    pos = POSITION.COLLECTION_1;
                    break;
                case 1:
                    pos = POSITION.COLLECTION_2;
                    break;
                case 2:
                    pos = POSITION.PIXEL_1;
                    break;
                case 3:
                    pos = POSITION.PIXEL_2;
                    break;
            }
        }
    }
    public void update(){
            switch (pos) {
                case WAIT:
                    break;
                case COLLECTION_1:
                    servo.setPosition(pixel_1_collection);
                    delay.reset();
                    pos = POSITION.WAIT;
                    break;
                case COLLECTION_2:
                    servo.setPosition(pixel_2_collection);
                    delay.reset();
                    pos = POSITION.WAIT;
                    break;
                case PIXEL_1:
                    servo.setPosition(pixel_1_drop);
                    delay.reset();
                    pos = POSITION.WAIT;
                    break;
                case PIXEL_2:
                    servo.setPosition(pixel_2_drop);
                    delay.reset();
                    pos = POSITION.WAIT;
                    break;
            }

    }
}
