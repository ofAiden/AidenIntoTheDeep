package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.AidenRobot;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class AidenDriveHW extends LinearOpMode {

    @Override
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Globals.TESTING_DISABLE_CONTROL = false;

        Robot robot = new Robot(hardwareMap);
        Sensors sensors = new Sensors(robot);
        AidenRobot aidenRobot = new AidenRobot(hardwareMap );



        telemetry.addData("State", "READY TO START");
        telemetry.update();

        double x, y, angle, leftFrontPower, rightFrontPower, leftBackPower, rightBackPower, motormax;
        while (opModeInInit()) {
            robot.update();
        }
        while (opModeIsActive()){
            y = gamepad1.left_stick_x;
            x = -gamepad1.left_stick_y;
            angle = gamepad1.right_stick_x;
            motormax = Math.max(1.0,Math.abs(y)+Math.abs(x)+Math.abs(angle));

            leftFrontPower = (y + x + angle)/motormax;
            rightFrontPower = (y -x - angle)/motormax;
            leftBackPower = (y - x - angle)/motormax;
            rightBackPower = (y + x - angle)/motormax;

            aidenRobot.leftFront.setTargetPower(leftFrontPower);
            aidenRobot.leftBack.setTargetPower(leftBackPower);
            aidenRobot.rightFront.setTargetPower(rightFrontPower);
            aidenRobot.rightBack.setTargetPower(rightBackPower);
        }

    }
}