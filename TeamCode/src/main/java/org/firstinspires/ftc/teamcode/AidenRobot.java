package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.intake.AidenIntake;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.intake.AidenIntake;
import org.firstinspires.ftc.teamcode.subsystems.deposit.AidenDeposit;



public class AidenRobot {
    public PriorityMotor leftFront;
    public PriorityMotor leftBack;
    public PriorityMotor rightBack;
    public PriorityMotor rightFront;
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;
    public final AidenIntake intake;
    public final AidenDeposit deposit;


    public AidenRobot(HardwareMap hardwareMap, Sensors sensors) {
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();
        leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront", 3, 5, sensors);
        leftBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftBack"), "leftBack", 3, 5, sensors);
        rightBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightBack"), "rightBack", 3, 5, sensors);
        rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront", 3, 5, sensors);
        hardwareQueue.addDevice(leftFront);
        hardwareQueue.addDevice(leftBack);
        hardwareQueue.addDevice(rightFront);
        hardwareQueue.addDevice(rightBack);
        rightFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        rightBack.motor[0].setDirection(DcMotor.Direction.REVERSE);
        deposit = new AidenDeposit(this);
        intake = new AidenIntake(this);
    }

    public void update(){
        intake.update();
        deposit.update();
        hardwareQueue.update();
    }

}
