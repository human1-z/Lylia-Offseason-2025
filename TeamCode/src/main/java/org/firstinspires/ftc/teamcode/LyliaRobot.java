package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.LyliaDeposit;
import org.firstinspires.ftc.teamcode.subsystems.LyliaIntake;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class LyliaRobot {
    //create a hardware queue
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;
    public LyliaDrivetrain drivetrain;
    public LyliaDeposit deposit;
    public LyliaIntake intake;
    public Sensors sensors;

    public LyliaRobot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.hardwareQueue = new HardwareQueue();

        //initialize the drivetrain, depo, intake, and sensors
    }

    public void update(LyliaRobot lyliaRobot) {
        // call update() on all the subsystems
        lyliaRobot.update(lyliaRobot.lyliaDrivetrain);
    }
}