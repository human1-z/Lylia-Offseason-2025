package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.LyliaDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LyliaIntake;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LyliaRobot {
    public final HardwareMap hardwareMap;
    public final HardwareQueue hardwareQueue;
    public final LyliaDrivetrain drivetrain;
    public final LyliaDeposit deposit;
    public final LyliaIntake intake;
    public final Sensors sensors;
    public final Gamepad gamepad;

    public LyliaRobot(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap; // set the hardwareMap attribute of the Robot to the one that was passed in
        this.hardwareQueue = new HardwareQueue(); // hardware queue managed in the robot class

        drivetrain = new LyliaDrivetrain(this);
        deposit = new LyliaDeposit(this);
        intake = new LyliaIntake(this);
        sensors = new Sensors(this);
        gamepad = new Gamepad();
    }

    public void update(LyliaRobot robot) {
        this.hardwareQueue.update();
        sensors.update();
        drivetrain.update();
        intake.update();
        deposit.update();
    }
}