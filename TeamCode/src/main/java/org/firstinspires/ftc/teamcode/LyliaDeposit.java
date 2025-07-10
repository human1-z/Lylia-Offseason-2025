package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.AXON_MINI;
import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.PRO_MODELER;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class LyliaDeposit{
    public final nPriorityServo claw; // open/close claw
    public final nPriorityServo clawAngle; // rotate claw up/down
    public final nPriorityServo clawExtension; // extend/retract end effector
    public final nPriorityServo[] armRotation; // rotate arm

    // arbitrary numbers
    public static double clawOpenPosition = 500;
    public static double clawClosedPosition = 0;

    public final Robot robot;

    public enum State {
        IDLE,
        TRANSFER_OPEN, //vertical slides down, horizontl slides in, arm rotated certain position, end effector rotated 90 degreees down, claw open
        TRANSFER_CLOSED, //same as above but claw closed
        DEPOSIT_SAMPLE_WAIT, // arm rotated around facing other way, end effector extended all the way out, claw still closed
        DEPOSIT_PIXEL_WAIT,
        DEPOSIT_SAMPLE, //arm rotated around facing other way, end effector extended all the way out, claw opened
        DEPOSIT_PIXEL
    }

    public LyliaDeposit(Robot robot){
        this.robot = robot;

        claw = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "claw"), "claw", AXON_MINI, 0.25,0.75,0.5, new boolean[]{false}, 0.2, 0.4);
        clawAngle = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "clawAngle"), "clawAngle", PRO_MODELER, 0.0, 1.0, 0.5, new boolean[]{false}, 0.2, 0.5);
        clawExtension = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "clawExtension"), "clawExtension", AXON_MINI, 0.0, 1.0, 0.0, new boolean[]{false}, 0.2, 0.4);
        armRotation = new nPriorityServo[](new Servo[] {robot.hardwareMap.get(Servo.class, "armLeft"), robot.hardwareMap.get(Servo.class, "armRight")}, "armServos", AXON_MINI, 0, 1, 0, new boolean[] {false, true}, 0.4, 0.5);

        robot.hardwareQueue.addDevice(claw);
        robot.hardwareQueue.addDevice(clawAngle);
        robot.hardwareQueue.addDevice(clawExtension);
        robot.hardwareQueue.addDevice(armRotation);
    }

    public State state = State.IDLE;

    // methods
    public void clawOpen(double position){
        claw.setTargetPos(position);
    }
    public void clawClose(double position){
        claw.setTargetPos(position);
    }

    public void setClawAngle(double angle){
        clawAngle.setTargetAngle(angle);
    }

    public void update() {
        switch (state) {
            case IDLE:
                // vertical slides fully down
                // horizontal slides retracted?
                break;
            case TRANSFER_OPEN:
                // vertical slides fully down
                // horizontal slides retracted
                // arm rotated to be about 120 degrees from vertical
                // end effector (claw) rotated 90 degrees to arm
                // claw OPEN
                clawOpen(clawOpenPosition);
                break;
            case TRANSFER_CLOSED:
                // same as above, except
                // claw CLOSED
                clawClose(clawClosedPosition);
                break;
            case DEPOSIT_SAMPLE_WAIT:
                // verticl slides up
                //horizontal slides sitll retracted
                // arm rotated to be about 300 degrees from vertical
                // end effector rotated to be parallel to ground
                clawClose(clawClosedPosition); // claw CLOSED
                break;
            case DEPOSIT_PIXEL_WAIT:
                // vertical slides up, depending on how high pixel the poles are
                // horizontal slides still retracted
                // arm rotated to be parallel to the ground
                // end effector rotated to also be parallel to ground
                // claw closed
                break;
            case DEPOSIT_SAMPLE:
                //same as above, but
                clawOpen(clawOpenPosition);
            case DEPOSIT_PIXEL:
                //same as above, but
                clawClose(clawClosedPosition);

        }
    }
}