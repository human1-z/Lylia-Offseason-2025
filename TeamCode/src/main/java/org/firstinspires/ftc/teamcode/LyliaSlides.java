package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class LyliaSlides {
    public final PriorityMotor verticalSlides;
    public final Sensors sensors;
    public final Robot robot;

    public LyliaSlides(Robot robot){
        this.robot = robot;
        sensors = robot.sensors;

        verticalSlides = new PriorityMotor(new DcMotorEx[]{robot.hardwareMap.get(DcMotorEx.class, "verticalSlidesL"), robot.hardwareMap.get(DcMotorEx.class, "verticalSlidesR")}, "verticalSlides", 1, 1, new double[] {1, 1}, robot.sensors);
        robot.hardwareQueue.addDevice(verticalSlides);
    }
}