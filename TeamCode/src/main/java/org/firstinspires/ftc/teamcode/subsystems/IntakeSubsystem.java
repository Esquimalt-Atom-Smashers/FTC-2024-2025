package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.util.Range;

import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.Constants.PIDSubsystemState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import java.lang.Math;

public class IntakeSubsystem extends SubsystemBase {
    private HardwareMap hardwareMap;

    private final DcMotorEx leftArmJointMotor;
    private final DcMotorEx rightArmJointMotor;

    private final DcMotorEx linearSlideMotor;

    private final ServoEx wristServo;
    private final CRServo activeIntakeServo;

    private double speedMultiplier = 1.0;

    private double leftArmJointMotorTicks;
    private double rightArmJointMotorTicks;
    private double armJointMaximumAngleRotated;
    private double armJointMinimumAngleRotated;

    private double linearSlideMotorTicks;
    private double linearSlideDistanceTraveled;

    private final PIDController armJointController;
    private double armJointTarget = 0;
    private double armJointLastPower;

    private ElapsedTime armJointTimer;
    private double armJointTimeout;
    private PIDSubsystemState armJointState;

    private final PIDController linearSlideController;
    /** The state the arm/linear slide is in: (manual, moving-to-target, or at-target) */
    private double linearSlideTarget = 0;
    private double linearSlideLastPower;

    private ElapsedTime linearSlideTimer;
    private double linearSlideTimeout;
    private PIDSubsystemState linearSlideState;



    private final OpMode opMode;


    public IntakeSubsystem(HardwareMap hardwareMap,OpMode opMode){
        this.opMode= opMode;//a little fix so that the subsystem itself can add things into telemetry

        // intake arms motor creation
        leftArmJointMotor = hardwareMap.get(DcMotorEx.class,IntakeConstants.HORIZONTAL_SLIDE_LEFT_MOTOR_NAME);
        rightArmJointMotor = hardwareMap.get(DcMotorEx.class,IntakeConstants.HORIZONTAL_SLIDE_RIGHT_MOTOR_NAME);

        // intake arms direction
        leftArmJointMotor.setDirection(Constants.IntakeConstants.HORIZONTAL_SLIDE_LEFT_MOTOR_DIRECTION);
        rightArmJointMotor.setDirection(Constants.IntakeConstants.HORIZONTAL_SLIDE_RIGHT_MOTOR_DIRECTION);

        // brake checker if nothing is pressed
        leftArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideMotor = hardwareMap.get(DcMotorEx.class,IntakeConstants.LINEAR_SLIDE_MOTOR_NAME);
        linearSlideMotor.setDirection(Constants.IntakeConstants.LINEAR_SLIDE_MOTOR_DIRECTION);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlideController = new PIDController(Constants.IntakeConstants.LINEAR_SLIDE_P, IntakeConstants.LINEAR_SLIDE_I, IntakeConstants.LINEAR_SLIDE_D);
        linearSlideState = PIDSubsystemState.MANUAL;

        // wrist servo creation
        this.wristServo = new SimpleServo(hardwareMap,IntakeConstants.INTAKE_WRIST_SERVO_NAME, IntakeConstants.INTAKE_WRIST_SERVO_MIN_ANGLE,IntakeConstants.INTAKE_WRIST_SERVO_MAX_ANGLE);
        this.activeIntakeServo = hardwareMap.get(CRServo.class, IntakeConstants.ACTIVE_INTAKE_SERVO_NAME);

        armJointController = new PIDController(IntakeConstants.ARM_JOINT_P, IntakeConstants.ARM_JOINT_I, IntakeConstants.ARM_JOINT_D);


        resetEncoders();

    }

    public void updateIntakeSubsystemDistance() {
        leftArmJointMotorTicks = leftArmJointMotor.getCurrentPosition();
        rightArmJointMotorTicks = rightArmJointMotor.getCurrentPosition();

        linearSlideMotorTicks = linearSlideMotor.getCurrentPosition();
    }

    public void resetEncoders() {
        // reset and stop arm motors
        leftArmJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // start arm motors
        leftArmJointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightArmJointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void changeIntakeSubsystemMotorSpeedMultiplier(){
        if(speedMultiplier == 1){
            speedMultiplier = 0.1;
        } else{
            speedMultiplier = 1;
        }
    }

    public double getArmJointDegreeRotated(){
        if(leftArmJointMotorTicks >= rightArmJointMotorTicks){
            armJointMaximumAngleRotated = (leftArmJointMotorTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * 360;
            armJointMinimumAngleRotated = (rightArmJointMotorTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * 360;
        } else{
            armJointMaximumAngleRotated = (rightArmJointMotorTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * 360;
            armJointMinimumAngleRotated = (leftArmJointMotorTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * 360;
        }

        return (armJointMaximumAngleRotated + armJointMinimumAngleRotated) / 2;
    }

    public void runArmJointWithPID(double targetPosition, double timeout) {
        //set target and linearSlideTimeout for pid system
        this.armJointTarget = targetPosition;
        double initialDegreeRotated = getArmJointDegreeRotated();
        armJointState = PIDSubsystemState.MOVING_TO_TARGET;
        if (armJointTimer == null) armJointTimer = new ElapsedTime();
        else armJointTimer.reset();
        armJointTimeout = timeout;
        // If we aren't at the target
            while (armJointState == PIDSubsystemState.MOVING_TO_TARGET){
                // If we meet the position(exceed or under)
                // stop and reset the encoders
                if(armJointTarget > initialDegreeRotated){
                    if (this.armJointTarget <= armJointMinimumAngleRotated) {
                        leftArmJointMotor.setPower(0.0);
                        rightArmJointMotor.setPower(0.0);
                        armJointState = PIDSubsystemState.AT_TARGET;
                        return;
                    }
                }
                else{
                    if (this.armJointTarget >= armJointMaximumAngleRotated) {
                        leftArmJointMotor.setPower(0.0);
                        rightArmJointMotor.setPower(0.0);
                        armJointState = PIDSubsystemState.AT_TARGET;
                        return;
                    }
                }
                // Calculate how much we need to move the motor by
                armJointController.setPID(IntakeConstants.ARM_JOINT_P, IntakeConstants.ARM_JOINT_I, IntakeConstants.ARM_JOINT_D);
                double armJointPosition = getArmJointDegreeRotated();
                double power = linearSlideController.calculate(armJointPosition, this.armJointTarget);
                armJointLastPower = power;
                leftArmJointMotor.setPower(power);
                rightArmJointMotor.setPower(power);
                // If the power we are setting is basically none, we are close enough to the target
                if (Math.abs(power) <= Constants.IntakeConstants.ARM_JOINT_PID_POWER_TOLERANCE || isArmJointTimeoutPassed(armJointTimeout,armJointTimer)) {
                    armJointState = PIDSubsystemState.AT_TARGET;
                    leftArmJointMotor.setPower(0.0);
                    rightArmJointMotor.setPower(0.0);

            }
        }
    }


    /**
     * Check if the timeout has passed, if is has, reset the timeout and return true.
     *
     * @return True if the timeout has elapsed, false otherwise
     */
    private boolean isArmJointTimeoutPassed(double timeout,ElapsedTime armJointTimer) {
        if (timeout > 0 && armJointTimer.seconds() >= timeout) {
            this.armJointTimeout = 0;
            return true;
        }
        return false;
    }

    public double getLinearSlideDistanceTraveled() {
        linearSlideDistanceTraveled = (linearSlideMotorTicks / Constants.MOTOR_TICKS_PER_REVOLUTION) * IntakeConstants.LINEAR_SLIDE_PULLEY_CIRCUMFERENCE;

        return linearSlideDistanceTraveled;
    }

    /*
     * gets input from controllers and retracts/extends the arms
     */
    public void runArmJoints(double input){

        input = Math.abs(input) >= Constants.DriveConstants.DEADZONE ? input : 0;

        if (armJointMaximumAngleRotated >= IntakeConstants.MAXIMUM_ANGLE_ROTATED) {
            input = -Math.abs(input);
        }
        if(armJointMinimumAngleRotated <= IntakeConstants.MINIMUM_ANGLE_ROTATED){
            input = Math.abs(input);
        }

        leftArmJointMotor.setPower(Range.clip(input, -1, 1) * speedMultiplier);
        rightArmJointMotor.setPower(Range.clip(input, -1, 1) * speedMultiplier);
    }

    //run by manual input
    public void runLinearSlideManually(double input){
        input =Math.abs(input) >= Constants.DriveConstants.DEADZONE ? input : 0;

        linearSlideMotor.setPower(Range.clip(input, -1, 1) * speedMultiplier);
    }

    /**
     * Use the PID controller to calculate how fast we should set the motor to. If the motor is moving slow enough,
     * we are close enough and stop moving further.
     */

    public void runLinearSlideWithPID(double targetPosition, double timeout) {
        //set target and linearSlideTimeout for pid system
        this.linearSlideTarget = targetPosition;
        double initialSlideDistanceTraveled = linearSlideDistanceTraveled;
        linearSlideState = PIDSubsystemState.MOVING_TO_TARGET;
        if (linearSlideTimer == null) linearSlideTimer = new ElapsedTime();
        else linearSlideTimer.reset();
        linearSlideTimeout = timeout;
        // If we aren't at the target
        while (linearSlideState == PIDSubsystemState.MOVING_TO_TARGET)
        {

            if(linearSlideTarget > initialSlideDistanceTraveled){
                if (this.linearSlideTarget <= linearSlideDistanceTraveled) {
                    leftArmJointMotor.setPower(0.0);
                    rightArmJointMotor.setPower(0.0);
                    armJointState = PIDSubsystemState.AT_TARGET;
                    return;
                }
            }
            else{
                if (this.linearSlideTarget >= linearSlideDistanceTraveled) {
                    leftArmJointMotor.setPower(0.0);
                    rightArmJointMotor.setPower(0.0);
                    armJointState = PIDSubsystemState.AT_TARGET;
                    return;
                }
            }
            // Calculate how much we need to move the motor by
            linearSlideController.setPID(IntakeConstants.LINEAR_SLIDE_P, IntakeConstants.LINEAR_SLIDE_I, IntakeConstants.LINEAR_SLIDE_D);
            double linearSlidePosition = getLinearSlideDistanceTraveled();
            double power = linearSlideController.calculate(linearSlidePosition, this.linearSlideTarget);
            linearSlideLastPower = power;
            linearSlideMotor.setPower(power);
            // If the power we are setting is basically none, we are close enough to the target
            if (Math.abs(power) <= Constants.IntakeConstants.LINEAR_SLIDE_PID_POWER_TOLERANCE || isLinearSlideTimeoutPassed(linearSlideTimeout,linearSlideTimer)) {
                    linearSlideState = PIDSubsystemState.AT_TARGET;
                    linearSlideMotor.setPower(0.0);
                }
            }
        }


    /**
     * Check if the timeout has passed, if is has, reset the timeout and return true.
     *
     * @return True if the timeout has elapsed, false otherwise
     */
    private boolean isLinearSlideTimeoutPassed(double timeout,ElapsedTime linearSlideTimer) {
        if (timeout > 0 && linearSlideTimer.seconds() >= timeout) {
            linearSlideTimeout = 0;
            return true;
        }
        return false;
    }

    /** @return True if the motor is at the target, false otherwise */
    public boolean isAtTarget() {
        return linearSlideState == PIDSubsystemState.AT_TARGET;
    }

    public double getLinearSlideLastPower(){
        return linearSlideLastPower;
    }

    public double getLinearSlideTarget(){
        return this.linearSlideTarget;
    }


    public void ActiveIntakeServoIn(){
        this.activeIntakeServo.setPower(1.0);
    }

    public void ActiveIntakeServoOut(){
        this.activeIntakeServo.setPower(-1.0);
    }

    public void stopActiveIntakeServo(){
        this.activeIntakeServo.setPower(0.0);
    }

    /*
    moves the wrist servo into position to retract
    */
    public void servoUpPosition(){
        wristServo.turnToAngle(IntakeConstants.INTAKE_WRIST_SERVO_UP_POSITION);
    }

    /*
    moves the wrist servo into position to pick up samples
     */
    public void servoDownPosition(){
        wristServo.turnToAngle(IntakeConstants.INTAKE_WRIST_SERVO_DOWN_POSITION);
    }


}
