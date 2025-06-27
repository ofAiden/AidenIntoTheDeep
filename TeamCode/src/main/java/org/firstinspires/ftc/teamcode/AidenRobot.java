package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.sensors.Sensors;


public class AidenRobot {
    public PriorityMotor leftFront;
    public PriorityMotor leftBack;
    public PriorityMotor rightBack;
    public PriorityMotor rightFront;
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;

    public AidenRobot(HardwareMap hardwareMap, Sensors sensors) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();
        leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront", 3, 5, sensors);
        leftBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftBack"), "leftRear", 3, 5, sensors);
        rightBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightBack"), "rightRear", 3, 5, sensors);
        rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront", 3, 5, sensors);
    }
}