package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class AidenDeposit {

    private PriorityMotor turret, vslide1, vslide2;
    private nPriorityServo extendo1, extendo2, vbar1, vbar2, claw, wrist;
    private AidenRobot robot;

    public AidenDeposit(AidenRobot robot) {
        this.robot = robot;
        vslide1 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vslide1"), "vslide1", 4, 5, null);
        vslide2 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vslide2"), "vslide2", 4, 5, null);
        turret = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "turret"), "turret", 3.5, 5, null);
        claw = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "claw")}, "claw", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        wrist = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "wrist")}, "wrist", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        extendo1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo1")}, "extendo1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 4, 5);
        extendo2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo2")}, "extendo2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 4, 5);
        vbar1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar1")}, "vbar1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        vbar2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar2")}, "vbar2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        robot.hardwareQueue.addDevices(vslide1,vslide2,turret,extendo1,extendo2,vbar1,vbar2,claw,wrist);
    }

    public void update() {


    }

}