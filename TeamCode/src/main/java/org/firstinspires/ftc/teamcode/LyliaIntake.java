package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.AXON_MINI;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class LyliaIntake{
    public final PriorityMotor intakeMotor;
    public final nPriorityServo intakeServo;
    public final Sensors sensors;
    public final Robot robot;

    public LyliaIntake(Robot robot){
        this.robot = robot;
        sensors = robot.sensors;

        intakeMotor = new PriorityMotor(robot.hardwareMap.get(DcMotorEx.class, "intake"), "intakeMotor", 1, 2, 1, sensors);
        intakeServo = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "intakeServo"), "intakeServo", AXON_MINI, 0.0, 1.0, 0.5, new boolean[] {false}, 0.2, 0.5);
    }
}