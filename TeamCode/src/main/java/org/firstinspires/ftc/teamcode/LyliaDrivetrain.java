package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.max;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


// drivetrain class is just a class that creates the drive motors (and encoders that dont get used)
// the way it will be used is by calling drivetrain.update or something
public class LyliaDrivetrain {

    public HardwareMap hardwareMap;
    public PriorityMotor lF, rF, lR, rR;
    public Gamepad gamepad;
    public double vertical, horizontal, rotation;

    public LyliaDrivetrain(Robot robot) {
        hardwareMap = robot.hardwareMap;
        gamepad = robot.gamepad;


        lF = new PriorityMotor(
                hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront",
                5.0, 1.0, null
        );

        robot.hardwareQueue.addDevice(lF);

        rF = new PriorityMotor(
                hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront",
                5.0, 1.0, null
        );

        robot.hardwareQueue.addDevice(rF);

        lR = new PriorityMotor(
                hardwareMap.get(DcMotorEx.class, "leftRear"), "leftRear",
                5.0, 1.0, null
        );

        robot.hardwareQueue.addDevice(lR);

        rR = new PriorityMotor(
                hardwareMap.get(DcMotorEx.class, "rightRear"), "rightRear",
                5.0, 1.0, null
        );

        robot.hardwareQueue.addDevice(rR);

    }

    public void update() {

        //negated y of left-stick because it seems like the controllers have it backwards in the first place
        vertical = -gamepad.left_stick_y;
        horizontal = gamepad.left_stick_x;
        rotation = gamepad.right_stick_x;

        double normDenominator = Math.max(vertical+horizontal+rotation, 1);

        //i could probably make these their own variables for finer control later on
        //theres *probably* a cleaner way to divide everything by normDenominator
        //but this works, and i doubt any performance increases will be big enough to matter


        lF.setTargetPower( (vertical + horizontal + rotation) / normDenominator );
        rF.setTargetPower( (vertical - horizontal - rotation) / normDenominator );
        lR.setTargetPower( (vertical - horizontal + rotation) / normDenominator );
        rR.setTargetPower( (vertical + horizontal - rotation) / normDenominator );

    }

}