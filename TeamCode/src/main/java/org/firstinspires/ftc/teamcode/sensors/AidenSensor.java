package org.firstinspires.ftc.teamcode.sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.AidenRobot;

public class AidenSensor {
    private final AidenRobot robot;
    private int turretEncoder;
    private int turretZero;
    public double turretAnglePerTick;

    private double voltage;
    //the time we want to have before updating each time
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    public AidenSensor(AidenRobot robot) {
        this.robot = robot;
    }
    public void update(){
        turretEncoder = ((PriorityMotor) robot.hardwareQueue.getDevice("turret")).motor[0].getCurrentPosition();
        if((System.currentTimeMillis()-lastVoltageUpdatedTime)>voltageUpdateTime){
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }
    }
    public double getTurretAngle(){
        return (turretEncoder-turretZero) * turretAnglePerTick;
    }
}
