package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class AidenIntake {

    private PriorityMotor intake, turret;
    private nPriorityServo extendo1, extendo2, vbar;
    private AidenRobot robot;

    public static PID TurretPid = new PID(1, 0.5,0.5); //tune ts

    public double turret_error;
    public double turret_encoder_position;

    //private double intake_motor_speed = 1.0; //tune later

    public enum IntakeStates {
        EXTENSION,
        INTAKEING,
        IDLEING,
        REVERSE
    }

    public IntakeStates intakeStates = IntakeStates.IDLEING;

    public AidenIntake(AidenRobot robot) {
        this.robot = robot;
        intake = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "intake"), "intake", 2, 5, null);
        turret = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "turret"), "turret", 2.5, 5, null);
        extendo1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo1")}, "extendo1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        extendo2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "extendo2")}, "extendo2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        vbar = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar1"), robot.hardwareMap.get(Servo.class, "vbar2")}, "vbar1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false, true}, 2, 5);
        robot.hardwareQueue.addDevices(intake,turret,extendo1,extendo2,vbar);
    }


    public void update() {

        switch (intakeStates) {
            case EXTENSION:
                //dostuff
                break;

            case INTAKEING:
                break;

            case IDLEING:
                //turn off motor
                break;
            case REVERSE:
                //reverse motor
                break;

            }
        }

        public void set_intake_motor(double speed) {
            intake.setTargetPower(speed);
        }

        public void set_vbar_position(double position_rad, double power) {
            vbar.setTargetAngle(position_rad, power);
        }

        public void set_turret_position (double position, double power) {
            //pull encoder position
            turret_error = position - turret_encoder_position;
            turret.setTargetPosition();
        }








    }




