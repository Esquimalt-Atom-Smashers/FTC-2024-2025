package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import java.lang.Math;

public class IntakeSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;

    private final Motor horizontalSlideLeftMotor;
    private final Motor horizontalSlideRightMotor;

    private final ServoEx wristServo;
    private final Motor activeIntakeServo;

    private enum intakeSubsystemState{
        EXTENDING,
        GETTING_BLOCK,
        RETRACTING,
        RELEASING_BLOCK
    }

    public IntakeSubsystem(HardwareMap hardwareMap){
        horizontalSlideLeftMotor = hardwareMap.get(Motor.class,IntakeConstants.HORIZONTAL_SLIDE_LEFT_MOTOR_NAME);
        horizontalSlideRightMotor = hardwareMap.get(Motor.class,IntakeConstants.HORIZONTAL_SLIDE_RIGHT_MOTOR_NAME);
        MotorGroup horizontalSlideMotors = new MotorGroup(horizontalSlideLeftMotor,horizontalSlideRightMotor);

        wristServo = new SimpleServo(hardwareMap,IntakeConstants.INTAKE_WRIST_SERVO_NAME, IntakeConstants.INTAKE_WRIST_SERVO_MIN_ANGLE,IntakeConstants.INTAKE_WRIST_SERVO_MAX_ANGLE);
        activeIntakeServo = new Motor(hardwareMap,IntakeConstants.ACTIVE_INTAKE_SERVO_NAME);//Have error
        // I dunno how to do this,leave it later

        horizontalSlideMotors.setRunMode(Motor.RunMode.PositionControl);
    }


}
