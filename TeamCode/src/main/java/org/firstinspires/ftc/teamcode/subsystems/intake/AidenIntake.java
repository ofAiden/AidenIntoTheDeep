package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;


public class AidenIntake {

    private final PriorityMotor extendo;
    private final PriorityMotor intake_motor;
    private final nPriorityServo extendo1, extendo2, vbar;
    private final AidenRobot robot;

    public static PID extendo_pid = new PID(1, 0, 0); //tune ts

    private double extendo_target_pos;
    private double extendo_current_pos = 0;




    public enum IntakeStates {
        START,
        EXTEND,
        DROP_DOWN,
        INTAKE,
        CONFIRM,
        RETRACT,
        TRANSFER,
        IDLE,
        REVERSE
    }

    public IntakeStates intakeStates = IntakeStates.START;

    public AidenIntake(AidenRobot robot) {
        this.robot = robot;
        extendo = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "extendo"), "extendo", 2, 5, null);
        intake_motor = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "intake_motor"), "intake_motor", 2.5, 5, null);
        extendo1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo1")}, "extendo1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        extendo2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo2")}, "extendo2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        vbar = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar1"), robot.hardwareMap.get(Servo.class, "vbar2")}, "vbar1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false, true}, 2, 5);
        robot.hardwareQueue.addDevices(intake_motor, extendo, extendo1, extendo2, vbar);
    }


    public void update() {
        update_extendo();
        switch(intakeStates) {
            case START:
                break;

            case EXTEND:

        }


    }

    public void update_extendo() {
        extendo_current_pos = robot.sensor.get_extendo_pos();
        double pow = 0;
        pow = extendo_pid.update(extendo_target_pos - extendo_current_pos, -0.7, 0.7);
        extendo.setTargetPower(pow);
    }


}




