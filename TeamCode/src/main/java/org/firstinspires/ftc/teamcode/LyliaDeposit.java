package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.AXON_MINI;
import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.PRO_MODELER;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class LyliaDeposit{
    public final nPriorityServo clawPosition;
    public final nPriorityServo clawRotate;

    public final Robot robot;

    public LyliaDeposit(Robot robot){
        this.robot = robot;
        clawPosition = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "clawPosition"), "clawPosition", AXON_MINI, 0.25,0.75,0.5, new boolean[] {false}, 0.2, 0.4);
        clawRotate = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "clawRotate"), "clawRotate", PRO_MODELER, 0.0, 1.0, 0.5, new boolean[] {false}, 0.2, 0.5);

        robot.hardwareQueue.addDevice(clawPosition);
        robot.hardwareQueue.addDevice(clawRotate);
    }

    public void setClawAngle(double angle){
        clawRotate.setTargetAngle(angle);
    }
    public void setClawPosition(double position){
        clawPosition.setTargetPos(position);
    }
}