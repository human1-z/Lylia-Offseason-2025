package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class LyliaTeleop extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        HardwareQueue hardwareQueue = new HardwareQueue();

        Robot robot = new Robot(hardwareMap);
        Gamepad gamepad1 = new Gamepad();
        Sensors sensors = new Sensors(robot); //ignore error; create a new sensors class

        PriorityMotor leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront", 0,7, 1, sensors);
        PriorityMotor leftBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftBack"), "leftBack", 0.7, 1, 1, sensors);
        PriorityMotor rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront", 0.7, 1, 1, sensors);
        PriorityMotor rightBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightBack"), "rightBack", 1, 1, 1, sensors);

        PriorityMotor[] motors = {leftFront, leftBack, rightFront, rightBack};

        for (int i=0; i<motors.length; i++) {
            hardwareQueue.addDevice(motors[i]);
        }

        double drive, strafe, turn;
        double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

        waitForStart();

        while (opModeIsActive()) {
            hardwareQueue.update();

            double denominator;

            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            denominator = Math.max(1, (Math.abs(drive) + Math.abs(strafe) + Math.abs(turn)));

            leftFrontPower = (drive + strafe + turn) / denominator;
            leftBackPower = (drive - strafe + turn) / denominator;
            rightFrontPower = (drive - strafe - turn) / denominator;
            rightBackPower = (drive + strafe - turn) / denominator;

            leftFront.setTargetPower(leftFrontPower);
            leftBack.setTargetPower(leftBackPower);
            rightFront.setTargetPower(rightFrontPower);
            rightBack.setTargetPower(rightBackPower);
        }
    }
}