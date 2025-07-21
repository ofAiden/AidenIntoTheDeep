package org.firstinspires.ftc.teamcode.subsystems.hang;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.subsystems.deposit.AidenDeposit;

public class Aidenhang {

    private AidenRobot robot;
    private AidenDeposit deposit;
    private nPriorityServo pto;

    public double pi = Math.PI;

    //variables that should be found when the robot is built
    public double hang_height;
    public double engage_pto;
    public double disengage_pto;

    public Aidenhang(AidenRobot robot) {
        this.robot = robot;
        pto = new nPriorityServo(new Servo[]{robot.hardwareMap.get(Servo.class, "wrist")}, "wrist", nPriorityServo.ServoType.AXON_MAX, 0, 1, 0.5, new boolean[]{false}, 3, 5);
        robot.hardwareQueue.addDevices(pto);
    }

    public enum HangStates{
        IDLE,
        HANG_READY,
        HANG_RETRACT,
        HANG,
        HANG_STOP
    }
    public HangStates hangStates = HangStates.IDLE;

    public void update(){
        switch (hangStates){
            case IDLE:
                set_pto(false);
                //no clue
                break;

            case HANG_READY:
                set_pto(true);
                if(deposit.hang_ready){
                    hangStates = HangStates.HANG;
                }
                break;

            case HANG:
                robot.rightBack.setTargetPower(1);
                robot.leftBack.setTargetPower(1);
                deposit.set_vslides_pos(hang_height);
                if(hang_height - deposit.vslides_current_pos<= 0.5){
                    hangStates = HangStates.HANG_STOP;
                }
                break;
            case HANG_STOP:
                robot.leftBack.setTargetPower(0);
                robot.rightBack.setTargetPower(0);
            case HANG_RETRACT:
                set_pto(false);
                deposit.set_vslides_pos(deposit.vslides_zero);
                break;
        }
    }

    public void set_pto(boolean state){
        if(state){
            pto.setTargetAngle(engage_pto);
        } else {
            pto.setTargetAngle(disengage_pto);
        }
    }
}
