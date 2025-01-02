package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.GamepadUtils;

public class LinearSlideSubsystem extends SubsystemBase {
    //CONSTANTS
    static final int MANUAL_CONTROL_RATE = 25;
    static final String LINEAR_SLIDE_MOTOR_NAME = "linearSlideMotor";
    static final DcMotorSimple.Direction LINEAR_SLIDE_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final int LINEAR_STARTING_POS = 0;
    static final double LINEAR_SLIDE_P = 0.005;
    static final double LINEAR_SLIDE_I = 0;
    static final double LINEAR_SLIDE_D = 0;
    static final double PID_SAFE_POWER = 0.0;
    static final int MINIMUM_SLIDE_POS = 0;
    static final int MAXIMUM_SLIDE_POS = 2900;

    private final DcMotorEx motor;

    private final HardwareMap hardwareMap;
    private final OpMode opMode;
    private final Telemetry telemetry;

    private final PIDController controller;

    private int targetPosition;
    private boolean operatorOverride;

    private ElapsedTime elapsedTime;
    double timeOut = 2; //seconds


    public LinearSlideSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        motor = hardwareMap.get(DcMotorEx.class, LINEAR_SLIDE_MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(LINEAR_SLIDE_MOTOR_DIRECTION);

        targetPosition = LINEAR_STARTING_POS;

        controller = new PIDController(LINEAR_SLIDE_P, LINEAR_SLIDE_I, LINEAR_SLIDE_D);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        operatorOverride = false;

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    public void runPIDPosition() {
        double outputPower = controller.calculate(motor.getCurrentPosition(), targetPosition);
        telemetry.addData("Power to motors:", outputPower);

        if(Math.abs(outputPower) > 0.6 ) {
            if (elapsedTime.seconds() > timeOut) {
                motor.setPower(PID_SAFE_POWER);
                telemetry.addLine("slide timeout triggered"); }
            else motor.setPower(outputPower); 
        } else {
            elapsedTime.reset();
            motor.setPower(outputPower);
        }
    }

    /** Takes the input from a controller and converts it into a plus or minus targetposition based on the previous target position.
     * 
     * @param input -> Controller Input
     * @param rate -> Rate of change per revolution (ticks)
     * @return Position (ticks)
     */
    public int convertStickToTarget(double input, int rate) {
        input = GamepadUtils.deadzone(input);
        telemetry.addData("Target Position:", (int) (input * rate + targetPosition));
        return (int) (input * rate + targetPosition);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = Range.clip(targetPosition, MINIMUM_SLIDE_POS, MAXIMUM_SLIDE_POS);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    //Commands

    public static class RunPIDCommand extends CommandBase {
        private LinearSlideSubsystem linearSlideSubsystem;
        
        public RunPIDCommand(LinearSlideSubsystem linearSlideSubsystem) {
            this.linearSlideSubsystem = linearSlideSubsystem;
        }

        @Override
        public void execute() {
            linearSlideSubsystem.telemetry.addData("Slide Target position", linearSlideSubsystem.getTargetPosition());
            linearSlideSubsystem.telemetry.addData("Slide Current position", linearSlideSubsystem.getCurrentPosition());
            linearSlideSubsystem.runPIDPosition();
        }
    }
}