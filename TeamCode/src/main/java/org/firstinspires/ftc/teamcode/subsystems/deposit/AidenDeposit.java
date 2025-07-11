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
    private nPriorityServo vbar, claw, wrist, slide, arm;
    private AidenRobot robot;

    public static PID vslidesPID = new PID(1, 0,0); //needs to be tuned
    public static PID armslidesPID = new PID(1, 0,0); //needs to be tuned


    public double vslides_error;
    public double vslides_current_pos;

    public double armslides_error;
    public double armslides_current_pos = slide.getCurrentAngle();

    public double extendo_current_pos = robot.sensor.get_extendo_pos();
    public double vslides_deposit_pos;

    public double current_vslides;
    public double current_arm_angle;
    public double current_extendo;
    public double current_x;
    public double current_y;
    public double theta;
    public double slide_distance;

    public double pixel_y;
    public double pixel_x;
    public double sample_y;
    public double sample_x;



    public AidenDeposit(AidenRobot robot) {
        this.robot = robot;
        vslide1 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vslide1"), "vslide1", 4, 5, null);
        vslide2 = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "vslide2"), "vslide2", 4, 5, null);
        extendo = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "extendo"), "extendo", 4, 5, null);
        claw = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "claw")}, "claw", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        wrist = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "wrist")}, "wrist", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        vbar = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "vbar1"),robot.hardwareMap.get(Servo.class, "vbar2")}, "vbar", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{true, false}, 2, 5);
        slide = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "slide1"),robot.hardwareMap.get(Servo.class, "slide2")}, "slide", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{true, false}, 2, 5);
        arm = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "arm")}, "arm", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);

        robot.hardwareQueue.addDevices(vslide1,vslide2,extendo,vbar,claw,wrist,slide, arm);
    }

    public enum DepositStates {
        TRANSFER,
        DEPOSIT_SAMPLE,
        DEPOSIT_PIXEL,
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
            case DEPOSIT_SAMPLE:
                this.updateDepositIK(sample_x,sample_y);
                break;
            case DEPOSIT_PIXEL:
                this.updateDepositIK(pixel_x,pixel_y);
                break;

        }

    }
    public void set_vslides_pos(double target_pos, double power){
        vslides_current_pos = robot.sensor.get_vslides_pos();

        vslides_error = target_pos - vslides_current_pos;

        vslide1.setTargetPower(vslidesPID.update(vslides_error, -1, 1) * power);
        vslide2.setTargetPower(vslidesPID.update(vslides_error, -1, 1) * power);
    }

    public void set_armslides_pos(double target_pos, double power){
        armslides_current_pos = slide.getCurrentAngle();

        armslides_error = target_pos - armslides_current_pos;

        slide.setTargetPos(armslidesPID.update(armslides_error, -1, 1) * power,1);
    }
    public void set_wrist_angle(double target_angle, double power){ wrist.setTargetAngle(target_angle,power); }

    public void set_claw_pos(double target_angle) { claw.setTargetAngle(target_angle);}

    public void updateDepositIK(double target_x, double target_y) {
        // 1. Read actual actuator positions
        current_vslides = robot.sensor.get_vslides_pos();
        current_arm_angle = arm.getCurrentAngle();
        current_extendo = slide.getCurrentAngle();

        //2. Calculate the current X, and Y
        current_y = current_vslides + Math.sin(current_arm_angle) * current_extendo;
        current_x = Math.cos(current_arm_angle) * current_extendo;

        target_x -= current_x;
        target_y -= current_y;

        // 3. Recalculate IK from current state
        theta = Math.atan((target_y/2)/target_x);
        slide_distance = Math.sqrt((target_y/2)*(target_y/2) + target_x*target_x;

        // 4. Command actuators toward new targets
        if(Math.abs(current_y - target_y) > 0.5 && Math.abs(current_x - target_x) > 0.5){
            set_vslides_pos(target_y/2,1); //assuming that slides work by like doing a targety/2 after its movements.
            arm.setTargetPos(theta, 1); //assuming that the arm angle doesn't change
            set_armslides_pos(slide_distance,1); //assuming that the slides keep the previous, and add this onto it this is what i meant for the vslides too
        } else{
            set_claw_pos(1); //assuming that open claw is 1
        }
    }

}