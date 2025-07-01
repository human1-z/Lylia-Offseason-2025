package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.max;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


//drivetrain class is just a class that creates the drive motors (and encoders that dont get used)
//the way it will be used is by calling drivetrain.update or something
public class LyliaDrivetrain {
    HardwareMap hardwareMap = new HardwareMap();
    HardwareQueue hardwareQueue = new HardwareQueue();

    Sensors sensors = new Sensors();

    //create all 4 motors
    PriorityMotor leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront", 0,7, 1, sensors);
    PriorityMotor leftBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftBack"), "leftBack", 0.7, 1, 1, sensors);
    PriorityMotor rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront", 0.7, 1, 1, sensors);
    PriorityMotor rightBack = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightBack"), "rightBack", 1, 1, 1, sensors);

    // put them in a list to make adding to hardwareQueue easier
    PriorityMotor[] motors = {leftFront, leftBack, rightFront, rightBack};

    // NEED TO FIX: error
    for (int i=0; i<motors.length; i++) {
        hardwareQueue.addDevice(motors[i]);
        return;
    }

    public LyliaDrivetrain(LyliaRobot LyliaRobot) {

    }

    public void drive(Gamepad gamepad1){
        Gamepad gamepad1 = this.gamepad1;

        double drive, strafe, turn;
        double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;

        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        // NEED TO ADD: normalization of these values to take the max of them
        leftFrontPower = (drive + strafe + turn);
        leftBackPower = (drive - strafe + turn);
        rightFrontPower = drive - strafe - turn;
        rightBackPower = drive + strafe - turn;

        double highestPower = max(leftFrontPower, leftBackPower), max()


    }
    public void update() {
        leftFront.setTargetPower(leftFrontPower);
    }


    //also has a drive() method?? why do you
}
