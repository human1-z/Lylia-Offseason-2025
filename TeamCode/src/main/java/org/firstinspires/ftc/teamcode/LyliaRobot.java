package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.LyliaDeposit;
import org.firstinspires.ftc.teamcode.subsystems.LyliaDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LyliaIntake;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LyliaRobot {
    //create a hardware queue
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;
    public LyliaDrivetrain drivetrain;
    public LyliaDeposit deposit;
    public LyliaIntake intake;
    public Sensors sensors;
    public Gamepad gamepad;

    public LyliaRobot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();

        //initialize the drivetrain, depo, intake, and sensors
    }

    public void update(LyliaRobot lyliaRobot) {
        // call update() on all the subsystems

        drivetrain.update();
        intake.update();
    }
}