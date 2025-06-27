package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class LyliaDriveHW extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        // create 4 motors, leftFront, leftRear, rightFront, rightRear
        // read the PriorityMotor class if you don't know how to make one
        // remember hardware map exists :3
        PriorityMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront", 1, 1, 1, 0);
        PriorityMotor leftBack = hardwareMap.get(DcMotorEx.class, "leftBack", 1, 1, 1, 0);
        PriorityMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront", 1, 1, 1, 0);
        PriorityMotor rightBack = hardwareMap.get(DcMotorEx.class, "rightBack", 1, 1, 1, 0);

        Gamepad gamepad1 = new Gamepad();
        double drive, strafe, turn;
        double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

        // init

        // going to need a while loop for updating
        // use the left and right joysticks
        // left is for magnitude of power applied
        // right is for turning

        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            leftFrontPower = drive + strafe + turn;
            leftBackPower = drive - strafe + turn;
            rightFrontPower = drive - strafe - turn;
            rightBackPower = drive + strafe - turn;

            //need to fix; max only takes 2 arguments
            double max = Math.max(1, Math.abs(leftFrontPower), Math.abs(leftBackPower), Math.abs(rightFrontPower), Math.abs(rightBackPower));

            leftFront.setTargetPower(leftFrontPower);
            leftBack.setTargetPower(leftBackPower);
            rightFront.setTargetPower(rightFrontPower);
            rightBack.setTargetPower(rightBackPower);

            //normalizing motor powers to fall between -1 and 1


            leftFront.update();
        }


    }
}
