package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


public class VerticalSlides {
    public final PriorityMotor verticalSlides;
    private final LyliaRobot robot;

    private double targetPosition;

    public VerticalSlides(LyliaRobot robot){
        robot.hardwareQueue.addDevice(verticalSlides);
        verticalSlides = new PriorityMotor(new DcMotorEx[]{robot.hardwareMap.get(DcMotorEx.class, "verticalSlidesL"), robot.hardwareMap.get(DcMotorEx.class, "verticalSlidesR")}, "verticalSlides", 1, 1, new double[] {0, 1}, robot.sensors);
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        // need to actually use the target position when updating
    }

    public void update() {

    }
}