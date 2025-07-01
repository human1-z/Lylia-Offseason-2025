package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.AXON_MAX;
import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.AXON_MINI;
import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.PRO_MODELER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class LyliaDeposit (){
    HardwareMap hardwareMap = new HardwareMap();

    public final nPriorityServo lyliaClawPosition;
    public final nPriorityServo lyliaClawRotate;

    public LyliaDeposit(LyliaRobot robot){
        lyliaClawPosition = new nPriorityServo(hardwareMap.get(Servo[].class, "clawPosition"), "lyliaClawPosition", AXON_MINI, 0.25,0.75,0.5, new boolean[] {false}, 0.2, 0.4);
        lyliaClawRotate = new nPriorityServo(hardwareMap.get(Servo[].class, "clawRotate"), "lyliaClawRotate", PRO_MODELER, 0.0, 1.0, 0.5, new boolean[] {false}, 0.2, 0.5);

        robot.hardwareQueue.addDevice(lyliaClawPosition);
        robot.hardwareQueue.addDevice(lyliaClawRotate);
    }

    public void setClawAngle(double angle){
        lyliaClawRotate.setTargetAngle(angle);
    }
    public void setClawPosition(double position){
        lyliaClawPosition.setTargetPos(position);
    }
}