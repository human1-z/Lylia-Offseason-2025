package org.firstinspires.ftc.teamcode.subsystems.hang;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LyliaRobot;
import org.firstinspires.ftc.teamcode.subsystems.LyliaDrivetrain;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class SohumHang {
    private final PriorityMotor frontDrive;
    private final PriorityMotor backDrive;
    public final LyliaDrivetrain drivetrain;
    private final nPriorityServo ptoServo; // PriorityServo if not CR

    private double PTO_ENGAGE_POS = 1.0;
    private double PTO_DISENGAGE_POS = 0.0;

    public SohumHang(LyliaRobot robot) {
        // Map drivetrain motors (used as hang motors in PTO)
        drivetrain = new LyliaDrivetrain(robot);
        frontDrive = drivetrain.lF;
        backDrive = drivetrain.lR;
//        DcMotorEx left = robot.hardwareMap.get(DcMotorEx.class, "leftFront");
//        DcMotorEx right = robot.hardwareMap.get(DcMotorEx.class, "rightFront");
//
//        frontDrive = new PriorityMotor(left, "leftFront", 1, 2, 1, robot.sensors);
//        backDrive = new PriorityMotor(right, "leftBack", 1, 2, 1, robot.sensors);

        // Map PTO servo (engages gear/belt to hang)
        Servo[] pto = robot.hardwareMap.get(Servo[].class, "ptoServo");
        ptoServo = new nPriorityServo(
                pto,
                "ptoServo",
                nPriorityServo.ServoType.AXON_MINI,
                0, 1, 0,
                new boolean[] {false},
                0.2, 0.4
        );

        // Add to hardware queue
//        robot.hardwareQueue.addDevice(frontDrive);
//        robot.hardwareQueue.addDevice(backDrive);
        robot.hardwareQueue.addDevice(ptoServo);
    }

    // Engage the PTO mechanism (switch to hang mode)
    public void engagePTO() {
        ptoServo.setTargetPos(PTO_ENGAGE_POS);
    }

    // Disengage PTO (return to drive mode)
    public void disengagePTO() {
        ptoServo.setTargetPos(PTO_DISENGAGE_POS);
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


