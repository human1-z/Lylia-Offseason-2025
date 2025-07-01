package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.LyliaDrivetrain;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class LyliaDriveHW extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // create 4 motors, leftFront, leftRear, rightFront, rightRear
        // read the PriorityMotor class if you don't know how to make one
        // remember hardware map exists :3
        HardwareMap hardwareMap = new hardwareMap();
        HardwareQueue hardwareQueue = new HardwareQueue();


        // going to need a while loop for updating
        // use the left and right joysticks
        // left is for magnitude of power applied
        // right is for turning

        waitForStart();

        while (opModeIsActive()) {
            LyliaDrivetrain.drive();
            //need to fix; max only takes 2 arguments
//            double max = Math.max(1, Math.abs(leftFrontPower), Math.abs(leftBackPower), Math.abs(rightFrontPower), Math.abs(rightBackPower));
//
            LyliaDrivetrain.leftFront.setTargetPower(leftFrontPower);
            leftBack.setTargetPower(leftBackPower);
            rightFront.setTargetPower(rightFrontPower);
            rightBack.setTargetPower(rightBackPower);

            hardwareQueue.update();
        }


    }
}
