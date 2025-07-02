package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.sensors.AidenSensor;


public class AidenIntake {

    private PriorityMotor intake, turret;
    private nPriorityServo extendo1, extendo2, vbar;
    private AidenRobot robot;

    public static PID TurretPid = new PID(1, 0,0); //tune ts

    public double turret_error;
    public double turret_encoder_position;

    public boolean deposit_in_position = false;

    //private double intake_motor_speed = 1.0; //tune later

    public enum IntakeStates {
        INTAKEING,
        IDLEING,
        REVERSE,
        WAIT_FOR_DEPOSIT,
        TRANSFER,
        GET_READY_TO_INTAKE
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
            case INTAKEING:
                if(deposit_in_position) {

                    //go to the point

                    set_intake_motor(1);
                    // if color sensor is read and works

                    intakeStates = IntakeStates.WAIT_FOR_DEPOSIT;

                } else {
                    intakeStates = IntakeStates.GET_READY_TO_INTAKE;
                }
                break;

            case IDLEING:
                //turn off motor
                set_intake_motor(0);
                break;

            case REVERSE:
                //reverse motor
                set_intake_motor(-1);
                break;


            case WAIT_FOR_DEPOSIT:
                //call deposit to do stuff

            case TRANSFER:
                // set_horizontal_extension();
                //call depsoit to deposit sample/pixel

            case GET_READY_TO_INTAKE:
                //move turret, vertical slides, horizontal slides
                set_turret_position(TargetPosition, 0.2);
                //set_vertical_extension(); fake vertical extension function
                //set_horizontal_extension(); fake vertical extension function

            }
        }

        public void set_intake_motor(double speed) {
            intake.setTargetPower(speed);
        }

        public void set_vbar_position(double position_rad, double power) {
            vbar.setTargetAngle(position_rad, power);
            //use a middle variable for all set methods
        }

        public void set_turret_position (double target_position, double power) {
            //pull encoder position

            target_position = Utils.minMaxClip(target_position, -Math.PI/3, Math.PI/3);
            turret_encoder_position = robot.sensor.getTurretAngle();

            turret_error = target_position - turret_encoder_position;

            turret.setTargetPower(TurretPid.update(turret_error, -1, 1) * power);
        }

    }




