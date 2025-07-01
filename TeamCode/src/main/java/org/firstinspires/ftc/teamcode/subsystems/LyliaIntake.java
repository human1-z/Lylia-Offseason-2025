package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class LyliaIntake () {
    HardwareMap hardwareMap = new HardwareMap();

    public final PriorityMotor intakeMotor;
    public final Sensors sensors;
    public final LyliaRobot robot;

    public LyliaIntake(LyliaRobot robot){
        this.robot = robot; //why do we need to
        sensors = robot.sensors;

        intakeMotor = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intake"), "intakeMotor", 1, 2, 1, sensors);
    }

}
