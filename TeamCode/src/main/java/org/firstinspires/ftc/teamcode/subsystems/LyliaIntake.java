package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.LyliaRobot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;


public class LyliaIntake {

    //basically i have no idea what each of the constants should be set to
    //that will come with testing of the robot
    //but i have placeholders
    //so if you see a constant that doesn't make sense, that's why

    public HardwareMap hardwareMap;
    public LyliaRobot robot;
    public PriorityMotor intakeMotor;
    public PriorityMotor extendoMotor;
    public nPriorityServo intakeRotServo;
    //just using the PID class that was already there
    public PID extendoPID;
    public double minError;
    public double err;

    public static double targetPos;

    public enum State {
        IDLE,
        RISING_EDGE_TRANSITION,
        INTAKE_ON,
        FALLING_EDGE_TRANSITION,
        TRANSFER_READY,
        REVERSE
    }

    public State intakeState = State.IDLE;

    public LyliaIntake(LyliaRobot robot) {
        this.robot = robot;
        hardwareMap = robot.hardwareMap;

        intakeMotor = new PriorityMotor(
                hardwareMap.get(DcMotorEx.class, "intakeMotor"), "intakeMotor",
                1.0, 1.0, robot.sensors
        );

        robot.hardwareQueue.addDevice(intakeMotor);

        extendoMotor = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "intakeExtensionMotor"),
                "intakeExtensionMotor",
                3, 5, 1, robot.sensors
        );
        robot.hardwareQueue.addDevice(extendoMotor);

        intakeRotServo = new nPriorityServo(
                new Servo[] {hardwareMap.get(Servo.class, "intakeRotServo")},
                "clawOpen",
                nPriorityServo.ServoType.AXON_MINI,
                1.0,
                1.0,
                1.0,
                new boolean[] {false},
                1.0,
                1.0
        );

        robot.hardwareQueue.addDevice(intakeRotServo);
    }

    public void update() {

        switch (intakeState) {

            case IDLE:
                //idle will literally be do nothing
                //transitions *to* idle are handled by falling-edge-trans
                //the only way to get out of idle is when an object is located
                break;

            case RISING_EDGE_TRANSITION:
                //getExtendoPos already exists within Sensors, so using that as placeholder
                err = Math.abs(robot.sensors.getExtendoPos()-targetPos);
                //-1 and 1 probably need to be tuned asw
                extendoMotor.setTargetPower(extendoPID.update(err, -1, 1));

                //dunno what this should really be
                intakeRotServo.setTargetAngle(Math.PI);

                if (err <= minError) {
                    intakeState = State.INTAKE_ON;
                }

                break;

            case INTAKE_ON:
                //now everything is in position, intake turns on
                //1.0 should be fine, but depends on how powerful it turns out to be
                intakeMotor.setTargetPower(1.0);

                //objectHeld() is just pseudocode
                if (objectHeld()) {
                    intakeState = State.FALLING_EDGE_TRANSITION;
                }

                break;

            case FALLING_EDGE_TRANSITION:
                //literally just RISING_EDGE_TRANSITION but in the opposite direction

                //no 'targetPos' bc its literally just 0
                err = Math.abs(robot.sensors.getExtendoPos());
                extendoMotor.setTargetPower(extendoPID.update(err, -1, 1));

                //dunno what this should really be
                intakeRotServo.setTargetAngle(0);

                if (err <= minError) {
                    intakeState = State.TRANSFER_READY;
                }

                break;

            case TRANSFER_READY:
                //idle but renamed so that Deposit knows that we have an item
                //all the setup for it was done in FALLING_EDGE_TRANSITION

                break;

            case REVERSE:
                //-1 should be fine, depends on how it works in practice
                intakeMotor.setTargetPower(-1);

                break;

        }

    }

    public boolean objectHeld() {
        //logic for this should change i think
        return Math.random() >= 0.5;
    }



}
