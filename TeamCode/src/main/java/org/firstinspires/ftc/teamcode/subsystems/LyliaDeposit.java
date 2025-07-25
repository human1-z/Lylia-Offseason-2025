package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.AXON_MINI;
import static org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo.ServoType.PRO_MODELER;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class LyliaDeposit{
    public nPriorityServo claw; // open/close claw
    public nPriorityServo clawRotServo; // rotate claw up/down
    public nPriorityServo armSlidesServo; // extend/retract end effector
    public nPriorityServo armRotServo; // rotate arm
    public VerticalSlides verticalSlides; //raise/lower vertical slides
    public LyliaIntake intake;
    public LyliaRobot robot;

    // claw open/close servo
    public static double
            CLAW_OPEN = Math.toRadians(50),
            CLAW_CLOSED = Math.toRadians(0);

    // claw rotation servo
    public static double
            CLAW_ANGLE_ZERO = Math.toRadians(0),
            CLAW_ANGLE_TRANSFER = Math.toRadians(45),
            CLAW_ANGLE_DEPOSIT = Math.toRadians(0),
            CLAW_ROTATE_POWER = 0.5;

    // arm rotation servo
    public static double
            ARM_ANGLE_ZERO = Math.toRadians(0),
            ARM_ANGLE_TRANSFER = Math.toRadians(45),
            ARM_ANGLE_PIXEL = Math.toRadians(270),
            ARM_ANGLE_SAMPLE = Math.toRadians(300),
            ARM_ROTATE_POWER = 0.5;

    // used by the arm extension servo
    // number in mm
    public static double
            ARM_EXTENSION_NONE = 0,
            ARM_EXTENSION_FULL = 100;

    // vertical slides motors
    public static double
            SLIDES_ZERO_POSITION = 0,
            SLIDES_TRANSFER_POSITION = 0, // ideally we minimize the number of parts we need to move for transfer, but this is here if it's needed
            SLIDES_DEPOSIT_PIXEL = 200,
            SLIDES_DEPOSIT_SAMPLE = 300;

    // end effector specifications for IK
    private final static double
            HEIGHT_OFFSET = 17.34828376,
            DISTANCE_OFFSET = 53.05625211,
            BAR1 = 73.00,
            BAR2 = 73.00; //ok apparently the two bars are actually the same length, so the math in setArmLength() did not have to be as complicated as it is


    public boolean depositNow = false;

    public enum State {
        IDLE,
        TRANSFER_OPEN, //vertical slides down, horizontal slides in, arm rotated certain position, end effector rotated 90 degrees down, claw open
        TRANSFER_CLOSED, //same as above but claw closed
        DEPOSIT_SAMPLE_WAIT, // arm rotated around facing other way, end effector extended all the way out, claw still closed
        DEPOSIT_PIXEL_WAIT,
        DEPOSIT_SAMPLE, //arm rotated around facing other way, end effector extended all the way out, claw opened
        DEPOSIT_PIXEL
    }

    public LyliaDeposit(LyliaRobot robot){
        this.robot = robot;

        claw = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "claw"), "claw", AXON_MINI, 0.25,0.75,0.5, new boolean[]{false}, 0.2, 0.4);
        clawRotServo = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "clawRotServo"), "clawRotServo", PRO_MODELER, 0.0, 1.0, 0.5, new boolean[]{false}, 0.2, 0.5);
        armSlidesServo = new nPriorityServo(robot.hardwareMap.get(Servo[].class, "armSlidesServo"), "armSlidesServo", AXON_MINI, 0.0, 1.0, 0.0, new boolean[]{false}, 0.2, 0.4);

        Servo armServoLeft = robot.hardwareMap.get(Servo.class, "armLeft");
        Servo armServoRight = robot.hardwareMap.get(Servo.class, "armRight");
        armRotServo = new nPriorityServo(new Servo[] {armServoLeft, armServoRight}, "armServos", AXON_MINI, 0, 1, 0, new boolean[] {false, true}, 0.4, 0.5);

        verticalSlides = new VerticalSlides(this.robot);
        intake = new LyliaIntake(this.robot);

        robot.hardwareQueue.addDevice(claw);
        robot.hardwareQueue.addDevice(clawRotServo);
        robot.hardwareQueue.addDevice(armSlidesServo);
        robot.hardwareQueue.addDevice(armRotServo);
    }

    public State state = State.IDLE;

    public void update() {
        switch (state) {
            case IDLE:
                setArmLength(ARM_EXTENSION_NONE); // retract the end effector
                setClawRotation(CLAW_ANGLE_ZERO);
                setArmRotServo(ARM_ANGLE_ZERO); // arm facing down, parallel to vertical slides
                clawClose(); // if the claw fits in the robot while it's open, we might not need this
                verticalSlides.setTargetLength(SLIDES_ZERO_POSITION); // retract vertical slides

                if (intake.intakeState == LyliaIntake.State.TRANSFER_READY &&
                        armSlidesServo.inPosition() &&
                        clawRotServo.inPosition() &&
                        armRotServo.inPosition() &&
                        claw.inPosition() &&
                        verticalSlides.inPosition(0.3)
                ) { state = State.TRANSFER_OPEN; }
                break;

            case TRANSFER_OPEN:
                verticalSlides.setTargetLength(SLIDES_TRANSFER_POSITION); // raise v slides
                setArmRotServo(ARM_ANGLE_TRANSFER); // raise the arm
                setClawRotation(CLAW_ANGLE_TRANSFER);
                clawOpen();

                if (verticalSlides.inPosition(0.3) && armRotServo.inPosition() && clawRotServo.inPosition() && claw.inPosition()) {
                    state = State.TRANSFER_CLOSED;
                }
                break;

            case TRANSFER_CLOSED:
                clawClose();

                /* need to create a method that returns a boolean based on whether it is a sample or a pixel */
                if (claw.inPosition()) {
                    if (isSample()){
                        state = State.DEPOSIT_SAMPLE_WAIT;
                    } else {
                        state = State.DEPOSIT_PIXEL_WAIT;
                    }
                }
                break;

            case DEPOSIT_SAMPLE_WAIT:
                verticalSlides.setTargetLength(SLIDES_DEPOSIT_SAMPLE);
                setArmRotServo(ARM_ANGLE_SAMPLE);
                setClawRotation(CLAW_ANGLE_DEPOSIT);

                if (depositNow && verticalSlides.inPosition(0.3) && armRotServo.inPosition() && clawRotServo.inPosition()) {
                    state = State.DEPOSIT_SAMPLE;
                }
                break;

            case DEPOSIT_PIXEL_WAIT:
                verticalSlides.setTargetLength(SLIDES_DEPOSIT_PIXEL);
                setArmRotServo(ARM_ANGLE_PIXEL);
                setClawRotation(CLAW_ANGLE_DEPOSIT);

                if (depositNow && verticalSlides.inPosition(0.3) && armRotServo.inPosition() && clawRotServo.inPosition()) {
                    state = State.DEPOSIT_PIXEL;
                }
                break;

            case DEPOSIT_SAMPLE:
                clawOpen();

                state = State.IDLE;
                break;

            case DEPOSIT_PIXEL:
                setArmLength(ARM_EXTENSION_FULL);
                clawOpen();

                state = State.IDLE;
                break;
        }

        verticalSlides.update();
        intake.update();
    }


    public void setClawRotation(double targetAngle) {
        clawRotServo.setTargetAngle(targetAngle, CLAW_ROTATE_POWER);
    }
    public void setArmRotServo(double targetAngle) {
        armRotServo.setTargetAngle(targetAngle, ARM_ROTATE_POWER);
    }
    public void clawOpen() {
        claw.setTargetAngle(CLAW_OPEN, 1);
    }
    public void clawClose() {
        claw.setTargetAngle(CLAW_CLOSED, 1);
    }
    public void setArmLength(double length) {
        double radians;
        radians = (Math.acos(((length + DISTANCE_OFFSET)*(length + DISTANCE_OFFSET) + HEIGHT_OFFSET*HEIGHT_OFFSET - BAR2*BAR2) / BAR1*BAR1)) / 2;
        armSlidesServo.setTargetAngle(radians);
    }

    public boolean isSample() {
       // use some sensors to tell whether the object held is a pixel or sample
       // color sensors or vision or touch sensors?
    }
}