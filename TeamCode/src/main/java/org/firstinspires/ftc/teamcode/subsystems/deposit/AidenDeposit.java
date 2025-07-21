package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.subsystems.intake.AidenIntake;


public class AidenDeposit {

    private AidenRobot robot;
    private AidenIntake intake;
    private PriorityMotor vslide,extendo;
    private DcMotorEx[] vslides = {robot.hardwareMap.get(DcMotorEx.class, "vslide1"), robot.hardwareMap.get(DcMotorEx.class, "vslide2")};
    private double[] vslide_multiplier = {1,-1};
    private nPriorityServo vbar, claw, wrist, slide, arm;


    public static PID vslidesPID = new PID(1, 0,0); //needs to be tuned
    public static PID armslidesPID = new PID(1, 0,0); //needs to be tuned

    //tune these variables for the positions that we find.

    public double pixel_y;
    public double pixel_x;
    public double sample_y;
    public double sample_x;
    public double transfer_pos;
    public double open_claw;
    public double close_claw;
    public double vslides_zero;
    public double armslides_zero;
    public double transfer_angle;
    public boolean hang_ready;


    public double vslides_error;
    public double vslides_current_pos;
    public double pi = Math.PI;

    public double armslides_error;
    public double armslides_current_pos = slide.getCurrentAngle();
    private double slide_ratio = 0.5;

    public double extendo_current_pos = robot.sensor.get_extendo_pos();
    public double vslides_deposit_pos;

    public double current_vslides;
    public double current_arm_angle;
    public double current_extendo;
    public double current_x;
    public double current_y;
    public double theta;
    public double slide_distance;
    private boolean deposit_ready;



    public AidenDeposit(AidenRobot robot) {
        this.robot = robot;
        vslide = new PriorityMotor(vslides, "vslide", 4, 5,vslide_multiplier, null);
        extendo = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "extendo"), "extendo", 4, 5, null);
        claw = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "claw")}, "claw", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 2, 5);
        wrist = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "wrist")}, "wrist", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        slide = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "slide")}, "slide", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{true, false}, 2, 5);
        arm = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "arm")}, "arm", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);

        robot.hardwareQueue.addDevices(vslide,extendo,vbar,claw,wrist,slide, arm);
    }

    public enum DepositStates {
        TRANSFER,
        DEPOSIT_SAMPLE,
        DEPOSIT_PIXEL,
        DEPOSIT_PIXEL_READY,
        DEPOSIT_SAMPLE_READY,
        TRANSFER_READY,
        IDLE,
        HANG_READY
    }

    public DepositStates depositStates = DepositStates.IDLE;

    public void update() {
        switch (depositStates) {
            case TRANSFER:
                this.set_vslides_pos(vslides_zero);
                this.set_claw_pos(close_claw);
                this.set_arm_angle(transfer_angle);
                break;
            case TRANSFER_READY:
                if(intake.transfer_ready == true){
                    //transfer_ready would be a boolean that is kept from the intake where only if the intake is ready to transfer we can go into the actions in this state.
                    this.set_vslides_pos(vslides_zero);
                    this.set_arm_angle(transfer_angle);
                    this.set_claw_pos(open_claw);
                    this.set_armslides_pos(transfer_pos);
                }
            case IDLE:
                this.set_vslides_pos(0);
                this.set_wrist_angle(Math.PI/4,1);
                this.set_claw_pos(open_claw);
                this.set_armslides_pos(armslides_zero);
                break;
            case DEPOSIT_SAMPLE_READY:
                while (!deposit_ready) {
                    this.updateDepositIK(sample_x,sample_y);
                }
                break;
            case DEPOSIT_PIXEL_READY:
                while (!deposit_ready) {
                    this.updateDepositIK(pixel_x,pixel_y);
                }
                break;
            case DEPOSIT_PIXEL:
                this.set_claw_pos(open_claw);
                break;
            case DEPOSIT_SAMPLE:
                this.set_claw_pos(open_claw);
                break;
            case HANG_READY:
                this.set_vslides_pos(vslides_zero);
                extendo_target_pos = 0;
                intake.update_extendo();
                this.set_arm_angle(pi);
                this.set_armslides_pos(armslides_zero);
                hang_ready = true;
        }

    }
    public void set_vslides_pos(double target_pos){
        vslides_current_pos = robot.sensor.get_vslides_pos();

        vslides_error = target_pos - vslides_current_pos;

        vslide.setTargetPower(vslidesPID.update(vslides_error, -1, 1));
    }

    public void set_armslides_pos(double target_pos){ slide.setTargetPos(target_pos);}
    public void set_wrist_angle(double target_angle, double power){ wrist.setTargetAngle(target_angle,power); }
    public void set_arm_angle(double target_angle){ arm.setTargetAngle(target_angle);}
    public void set_claw_pos(double target_angle) { claw.setTargetAngle(target_angle);}

    public void updateDepositIK(double target_x, double target_y) {
        // 1. Read actual actuator positions
        current_vslides = robot.sensor.get_vslides_pos();
        current_arm_angle = arm.getCurrentAngle();
        current_extendo = slide.getCurrentAngle();

        //2. Calculate the current X, and Y
        current_y = current_vslides + Math.sin(current_arm_angle) * current_extendo;
        current_x = Math.cos(current_arm_angle) * current_extendo;

        // 3. Recalculate positions from current state
        theta = Math.atan2((target_y-current_y),target_x-current_x);
        slide_distance = Math.sqrt(Math.pow(target_x-current_x,2) + Math.pow(target_y-current_y,2));

        // 4. Command actuators toward new targets
        if(Math.abs(current_y - target_y) > 0.5 && Math.abs(current_x - target_x) > 0.5){
            set_vslides_pos(target_y*slide_ratio); //assuming that slides work by like doing a targety/2 after its movements.
            arm.setTargetPos(theta, 1); //assuming that the arm angle doesn't change
            set_armslides_pos(slide_distance); //assuming that the slides keep the previous, and add this onto it this is what i meant for the vslides too
        } else {
            deposit_ready = true;
        }
    }

}