package org.firstinspires.ftc.teamcode.subsystems.hang;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class SohumHang {
    private final PriorityMotor frontDrive;
    private final PriorityMotor backDrive;
    private final PriorityCRServo ptoServo; // or PriorityServo if not CR

    public SohumHang(Robot robot) {
        // Map drivetrain motors (used as hang motors in PTO)
        DcMotorEx left = robot.hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx right = robot.hardwareMap.get(DcMotorEx.class, "rightFront");

        frontDrive = new PriorityMotor(left, "leftFront", 1, 2, 1, robot.sensors);
        backDrive = new PriorityMotor(right, "leftBack", 1, 2, 1, robot.sensors);

        // Map PTO servo (engages gear/belt to hang)
        CRServo pto = robot.hardwareMap.get(CRServo.class, "ptoServo");
        ptoServo = new PriorityCRServo(new CRServo[]{pto}, "ptoServo", 1, 2);

        // Add to hardware queue
        robot.hardwareQueue.addDevice(frontDrive);
        robot.hardwareQueue.addDevice(backDrive);
        robot.hardwareQueue.addDevice(ptoServo);
    }

    // Engage the PTO mechanism (switch to hang mode)
    public void engagePTO() {
        ptoServo.setTargetPower(1.0); // or use setPosition() if using regular Servo
    }

    // Disengage PTO (return to drive mode)
    public void disengagePTO() {
        ptoServo.setTargetPower(0.0);
    }

    // Start hanging using drivetrain motors
    public void autoHangUsingDrive() {
        engagePTO();
        frontDrive.setTargetPower(-1.0);  // Power both drive motors to lift robot
        backDrive.setTargetPower(-1.0);
    }

    // Stop the hanging process
    public void stopDriveHang() {
        frontDrive.setTargetPower(0.0);
        backDrive.setTargetPower(0.0);
    }


}


