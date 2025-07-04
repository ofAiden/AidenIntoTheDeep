package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.PID;


public class AidenDeposit {

    private PriorityMotor vslide1,vslide2,extendo;
    private nPriorityServo vbar1, vbar2, claw, wrist;
    private AidenRobot robot;

    public static PID vslidesPID = new PID(1, 0,0); //needs to be tuned

    public double vslides_error;
    public double vslides_current_pos;

    public double extendo_current_pos = robot.sensor.get_extendo_pos();
    public double vslides_deposit_pos;

    public double wrist_length;
    public double wrist_offset;



    public AidenDeposit(AidenRobot robot) {
        this.robot = robot;
        vslide1 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vslide1"), "vslide1", 4, 5, null);
        vslide2 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vslide2"), "vslide2", 4, 5, null);
        extendo = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "extendo"), "extendo", 4, 5, null);
        claw = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "claw")}, "claw", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        wrist = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "wrist")}, "wrist", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        vbar1 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar1")}, "vbar1", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        vbar2 = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar2")}, "vbar2", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        robot.hardwareQueue.addDevices(vslide1,vslide2,extendo,vbar1,vbar2,claw,wrist);
    }

    public enum DepositStates {
        TRANSFER,
        HOLD,
        DEPOSIT,
        IDLE
    }

    public DepositStates depositStates = DepositStates.IDLE;

    public void update() {
        switch (depositStates) {
            case TRANSFER:
                this.set_vslides_pos(0,1);
                this.set_claw_pos(Math.PI/2);
                this.set_wrist_angle(Math.PI/4,1);
                break;
            case IDLE:
                this.set_vslides_pos(0,1);
                this.set_wrist_angle(Math.PI/4,1);
                this.set_claw_pos(0);
                if(extendo_current_pos < 0.5){
                    depositStates = DepositStates.TRANSFER;
                }
                break;
            case HOLD:
                this.set_vslides_pos(vslides_deposit_pos,1);
                this.set_wrist_angle(-Math.PI/4,1);
                if(vslides_error > 0.5 ){
                    depositStates = DepositStates.DEPOSIT;
                }
                break;
            case DEPOSIT:

        }

    }
    public void set_vslides_pos(double target_pos, double power){
        vslides_current_pos = robot.sensor.get_vslides_pos();

        vslides_error = target_pos - vslides_current_pos;

        vslide1.setTargetPower(vslidesPID.update(vslides_error, -1, 1) * power);
    }
    public void set_wrist_angle(double target_angle, double power){ wrist.setTargetAngle(target_angle,power); }

    public void set_claw_pos(double target_angle) { claw.setTargetAngle(target_angle);}

    private double[] calculateIK(double deposit_height) {
        // Use current slide height to minimize movement
        double target_height= robot.sensor.get_vslides_pos();

        // Calculate wrist angle
        double sin_theta = (deposit_height - target_height) / wrist_length;
        double theta_w;
        if (Math.abs(sin_theta) > 1.0) {
            // Adjust slide height to maximize wrist range
           target_height= deposit_height - wrist_length * (sin_theta > 0 ? 1 : -1);
           sin_theta = (deposit_height - target_height) / wrist_length;
        }
        theta_w = Math.asin(sin_theta);


        // Adjust wrist angle for vertical pixel orientation
        double adjusted_theta_w = theta_w - wrist_offset;
        this.set_vslides_pos(target_height,1);
        this.set_wrist_angle(adjusted_theta_w,1);
    }


}