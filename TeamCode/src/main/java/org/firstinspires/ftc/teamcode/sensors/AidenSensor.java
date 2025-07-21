package org.firstinspires.ftc.teamcode.sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.AidenRobot;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class AidenSensor {
    private final AidenRobot robot;
    //sensor for specific things
    private int extendo_encoder;
    public static double extendoInchesPerTick = 19.0 / 467;
    private int extendoZero;

    private int vslide_encoder;
    public static double vslides_inches_per_tick;
    private int vslide_zero;

    private int armslide_encoder;
    public static double armslides_inches_per_tick;
    private int armslide_zero;

    public double turretAnglePerTick;

    private double voltage;
    //the time we want to have before updating each time
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    public AidenSensor(AidenRobot robot) {
        this.robot = robot;
    }
    public void update(){
        extendo_encoder = ((PriorityMotor) robot.hardwareQueue.getDevice("extendo")).motor[0].getCurrentPosition();
        vslide_encoder = ((PriorityMotor) robot.hardwareQueue.getDevice("vslide")).motor[0].getCurrentPosition();
        armslide_encoder = ((PriorityMotor) robot.hardwareQueue.getDevice("extendo")).motor[0].getCurrentPosition();
        if((System.currentTimeMillis()-lastVoltageUpdatedTime)>voltageUpdateTime){
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }
    }
    public double get_vslides_pos(){return(vslide_encoder-vslide_zero)*vslides_inches_per_tick;}
    public double get_extendo_pos() {
        return extendo_encoder * extendoInchesPerTick;
    }
    public double get_armslide_pos() {return(vslide_encoder-armslide_zero)* armslides_inches_per_tick;}
}
