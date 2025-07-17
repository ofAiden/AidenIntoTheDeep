package org.firstinspires.ftc.teamcode.subsystems.hang;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.PID;

public class Aidenhang {

    private AidenRobot robot;
    private PriorityMotor ;
    private nPriorityServo pto;
    private double[] vslide_multiplier = {1,-1};
    private DcMotorEx[] vslides = {robot.hardwareMap.get(DcMotorEx.class, "vslide1"), robot.hardwareMap.get(DcMotorEx.class, "vslide2")};

    public static PID vslidesPID = new PID(1, 0,0); //needs to be tuned

    public double vslides_error;
    public double vslides_current_pos;
    public double pi = Math.PI;

    //variables that should be found when the robot is built
    public double vslide_zero;
    public double hang_height;
    public boolean hang_ready = false;

    public Aidenhang(AidenRobot robot) {
        this.robot = robot;
        vslide = new PriorityMotor(vslides, "vslide", 4, 5,vslide_multiplier, null);
        robot.hardwareQueue.addDevices(vslide);
    }

    public enum HangStates{
        IDLE,
        HANG_READY,
        HANG_RETRACT,
        HANG
    }
    public HangStates hangStates = HangStates.IDLE;

    public void update(){
        switch (hangStates){
            case IDLE:
                //no clue
                break;

            case HANG_READY:
                if(hang_ready){
                    hangStates = HangStates.HANG
                }
                break;

            case HANG:
                set_vslides_pos(hang_height);
                if(hang_height - vslides_current_pos<= 0.5){
                    hangStates = HangStates.HANG_RETRACT;
                }
                break;

            case HANG_RETRACT:
                set_vslides_pos(vslide_zero);
                break;
        }
    }
    public void set_vslides_pos(double target_pos){
        vslides_current_pos = robot.sensor.get_vslides_pos();

        vslides_error = target_pos - vslides_current_pos;

        vslide.setTargetPower(vslidesPID.update(vslides_error, -1, 1));
    }
}
