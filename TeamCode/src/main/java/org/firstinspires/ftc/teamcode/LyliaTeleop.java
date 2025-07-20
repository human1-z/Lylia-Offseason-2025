package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.LyliaDrivetrain;

public class LyliaTeleop extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        HardwareQueue hardwareQueue = new HardwareQueue();

        Robot robot = new Robot(hardwareMap);
        Gamepad gamepad1 = new Gamepad();
        Sensors sensors = new Sensors(robot); //ignore error; create a new sensors class
        LyliaDrivetrain drivetrain = new LyliaDrivetrain(robot);

        waitForStart();

        while (opModeIsActive()) {
            hardwareQueue.update();
            robot.update();
        }
    }
}