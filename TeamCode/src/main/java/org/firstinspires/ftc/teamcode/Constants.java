package org.firstinspires.ftc.teamcode; 

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public abstract class Constants {
    //miscellaneous
    public static final int INCH_TO_TILE = 24;

    public static abstract class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
        public static final String BACK_LEFT_MOTOR_NAME = "rearLeftMotor";
        public static final String BACK_RIGHT_MOTOR_NAME = "rearRightMotor";
        public static final String IMU_NAME = "imu";

        public static final String LEFT_ENCODER_NAME = "leftEncoder";
        public static final String RIGHT_ENCODER_NAME = "rightEncoder";
        public static final String CENTER_ENCODER_NAME = "centerEncoder";

        public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final DcMotorSimple.Direction LEFT_ENCODER_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction RIGHT_ENCODER_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction CENTER_ENCODER_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ); //fixed orientation

        public static final RevHubOrientationOnRobot IMU_PARAMETERS_ROADRUNNER = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        public static final double DEADZONE = 0.1;
    }

    @Config
    public static abstract class IntakeConstants {
        public static final String LEFT_ARM_MOTOR_NAME = "leftArmJointMotor";
        public static final String RIGHT_ARM_MOTOR_NAME = "rightArmJointMotor";

        public static final DcMotorSimple.Direction LEFT_ARM_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction RIGHT_ARM_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static final double ARM_JOINT_P = 0.0088;
        public static final double ARM_JOINT_I = 0;
        public static final double ARM_JOINT_D = 0.00017;

        public static final String LINEAR_SLIDE_MOTOR_NAME = "linearSlideMotor";
        public static final DcMotorSimple.Direction LINEAR_SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

        public static final double LINEAR_SLIDE_P = 0.005;
        public static final double LINEAR_SLIDE_I = 0;
        public static final double LINEAR_SLIDE_D = 0;

        public static final String ACTIVE_INTAKE_SERVO_NAME = "activeIntakeServo";
        public static final String INTAKE_WRIST_SERVO_NAME = "intakeWristServo";

        public static int MANUAL_CONTROL_RATE = 25; //add final later

        public static final int MAXIMUM_SLIDE_POS = 2900;
        public static final int MINIMUM_SLIDE_POS = 0;

        public static final int MAXIMUM_ELBOW_POS = 770;
        public static final int MINIMUM_ELBOW_POS = 0;

        public static final int ELBOW_STARTING_POS = 115;
        public static final int LINEAR_STARTING_POS = 0;

        public static final double WRIST_OUTTAKE_POS = 0;
        public static final double WRIST_INTAKE_POS = 0.35;

    }
}