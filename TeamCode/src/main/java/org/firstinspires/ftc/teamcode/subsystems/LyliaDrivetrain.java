package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.max;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


// drivetrain class is just a class that creates the drive motors (and encoders that dont get used)
// the way it will be used is by calling drivetrain.update or something
public class LyliaDrivetrain {
    HardwareMap hardwareMap = new HardwareMap();
    HardwareQueue hardwareQueue = new HardwareQueue();

    LyliaRobot robot = new LyliaRobot(hardwareMap);


    // NEED TO FIX: error

    public LyliaDrivetrain(LyliaRobot robot) {
        
    }

    public void drive(Gamepad gamepad1){



    }
    public void update() {

        leftFront.setTargetPower(leftFrontPower);
    }


    //also has a drive() method?? why do you
}
