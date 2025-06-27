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
        AidenRobot aidenRobot = new AidenRobot(hardwareMap,sensors);



        telemetry.addData("State", "READY TO START");
        telemetry.update();

        while (opModeInInit()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double angle = gamepad1.right_stick_x;

            double leftFrontPower = y + x + angle;
            double rightFrontPower = y -x - angle;
            double leftBackPower = y - x - angle;
            double rightBackPower = y + x - angle;

            aidenRobot.leftFront.setTargetPower(leftFrontPower);
            aidenRobot.leftBack.setTargetPower(leftBackPower);
            aidenRobot.rightFront.setTargetPower(rightFrontPower);
            aidenRobot.rightBack.setTargetPower(rightBackPower);

            robot.update();
        }

        while (!isStopRequested()) {
            robot.drivetrain.drive(gamepad1, false);

            robot.update();

            telemetry.update();
        }

    }
}