package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


public class VerticalSlides {
    public final PriorityMotor verticalSlides;
    private final Robot robot;

    private double targetPosition;

    public VerticalSlides(Robot robot){
        verticalSlides = new PriorityMotor(new DcMotorEx[]{robot.hardwareMap.get(DcMotorEx.class, "verticalSlidesL"), robot.hardwareMap.get(DcMotorEx.class, "verticalSlidesR")}, "verticalSlides", 1, 1, new double[] {0, 1}, robot.sensors);
        robot.hardwareQueue.addDevice(verticalSlides);
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    public void update() {

    }
}