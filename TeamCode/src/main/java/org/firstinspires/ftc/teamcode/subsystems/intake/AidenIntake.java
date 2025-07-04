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
    private final nPriorityServo pivot;
    private final AidenRobot robot;

    public static PID extendo_pid = new PID(1, 0, 0); //tune ts

    private double extendo_target_pos;
    private double extendo_current_pos = 0;
    private double extendo_error;

    private double extendo_pow = 0;
    private double extendo_keep_in_pow = -0.2;




    public enum IntakeStates {
        EXTEND,
        INTAKE,
        CONFIRM,
        RETRACT,
        TRANSFER,
        IDLE,
        REVERSE
    }

    public IntakeStates intakeStates = IntakeStates.IDLE;

    public AidenIntake(AidenRobot robot) {
        this.robot = robot;
        extendo = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "extendo"), "extendo", 2, 5, null);
        intake_motor = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "intake_motor"), "intake_motor", 2.5, 5, null);
        pivot = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "pivot")}, "pivot", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        robot.hardwareQueue.addDevices(intake_motor, extendo, pivot);
    }


    public void update() {
        this.update_extendo();
        switch(intakeStates) {
            case IDLE:
                //idk how to handle this
            case EXTEND:
                extendo_target_pos = 20; //random number idk yet
                update_extendo();
                extendo_error = extendo_target_pos - extendo_current_pos;
                set_intake_pivot( Math.PI/5,1); // tune this value


                if(extendo_error < 0.5) {
                    intakeStates = IntakeStates.INTAKE;
                }

            case INTAKE:
                set_intake_motor_power(1); //tune this
                intakeStates = IntakeStates.CONFIRM;
            case CONFIRM:
                //if the color sensor detects the right block and its in, we can switch states to retract
                //otherwise go back to intake state
            case RETRACT:
                extendo_target_pos = 0;
                update_extendo();
                set_intake_pivot(Math.PI, 1); //tune this value

                if(get_extendo_pos() < 0.1 && pivot.inPosition()) {
                    intakeStates = IntakeStates.TRANSFER;
                }

            case TRANSFER:
                //do transfer stuff
            case REVERSE:
                set_intake_motor_power(-1); // tune this

        }


    }

    public void update_extendo() {
        if(extendo_current_pos > 0.5) {
            extendo_current_pos = robot.sensor.get_extendo_pos();
            extendo_error = extendo_target_pos - extendo_current_pos;
            extendo_pow = extendo_pid.update(extendo_error, -0.7, 0.7);
            extendo.setTargetPower(extendo_pow);
        } else {
            extendo.setTargetPower(extendo_keep_in_pow);
        }
    }

    public double get_extendo_pos() {
        return extendo_current_pos;
    }

    public void set_intake_pivot(double target_pos, double power) {
        pivot.setTargetAngle(target_pos, power);
    }

    public void set_intake_motor_power(double power) {
        intake_motor.setTargetPower(power);
    }


}




