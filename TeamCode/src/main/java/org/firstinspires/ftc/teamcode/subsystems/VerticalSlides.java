package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class VerticalSlides {
    public static PID pid = new PID(0.5, 0, 0);
    private double err;

    public final PriorityMotor vSlidesMotors;
    private final LyliaRobot robot;

    private double targetLength;
    public double length;

    public VerticalSlides(LyliaRobot robot){
        this.robot = robot;

        DcMotorEx vSlides1 = robot.hardwareMap.get(DcMotorEx.class, "vSlidesMotor0");
        DcMotorEx vSlides2 = robot.hardwareMap.get(DcMotorEx.class, "vSlidesMotor1");
        vSlidesMotors = new PriorityMotor(new DcMotorEx[] {vSlides1, vSlides2}, "vSlidesMotors", 1, 1, new double[] {-1, 1}, robot.sensors);

        robot.hardwareQueue.addDevice(vSlidesMotors);
    }

    public void update() {
        length = this.robot.sensors.getSlidesPos();
        err = targetLength - length;
        if (this.inPosition(0.3)){
            pid.resetIntegral();
        }
        vSlidesMotors.setTargetPower(pid.update(err, -1, 1));
    }

    public void setTargetLength(double length) {
        targetLength = length;
    }

    public boolean inPosition(double threshold) {
        return Math.abs(err) <= threshold;
    }
}