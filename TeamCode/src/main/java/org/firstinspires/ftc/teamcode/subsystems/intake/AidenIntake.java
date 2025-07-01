package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class AidenIntake {

    private PriorityMotor intake, turret;
    private nPriorityServo extendo1, extendo2, vbar1, vbar2;
    private AidenRobot robot;

    public AidenIntake(AidenRobot robot) {
        this.robot = robot;
        intake = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "intake"), "intake", 2, 5, null);
        turret = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "turret"), "turret", 2.5, 5, null);
        extendo1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo1")}, "extendo1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        extendo2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo2")}, "extendo2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        vbar1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar1")}, "vbar1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        vbar2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar2")}, "vbar2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        robot.hardwareQueue.addDevices(intake,turret,extendo1,extendo2,vbar1,vbar2);
    }

    public void extendSlides(){

    }
    public void update() {



    }

}