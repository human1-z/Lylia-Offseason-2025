package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Robot {
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;
    public LyliaDeposit deposit;
    public LyliaIntake intake;
    public Sensors sensors;

    public Robot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap; // set the hardwareMap attribute of the Robot to the one that was passed in
        this.hardwareQueue = new HardwareQueue(); // hardware queue managed in the robot class

        this.deposit = new LyliaDeposit(this);
        this.intake = new LyliaIntake(this);
        this.sensors=new Sensors(this); // ignore
    }

    public void update(Robot lyliaRobot) {
        // call update() on all the subsystems
    }

}
