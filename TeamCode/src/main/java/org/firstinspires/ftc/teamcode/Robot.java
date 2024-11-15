package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;
// RR-specific imports

import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {

    private final OpMode opMode;

    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    private final DriveSubsystem driveSubsystem;

    private final IntakeSubsystem intakeSubsystem;

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);
    
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);

        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap,opMode);
    }

    public void configureTeleOpBindings() {
        
        /* Controls:
        * Driver:
        *   Forward -> left y axis
        *   Strafe -> left x axis
        *   Turn -> right x axis
        *
        *   Reduce Speed -> right trigger
        *   Reset Gyro -> back button
        *   Enable/Disable Field Centric -> start button
        */ 

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
        defaultDriveCommand.addRequirements(driveSubsystem);

        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger speedVariationTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        speedVariationTrigger.whenActive(() -> driveSubsystem.changeSpeedMultiplier());//feedback from driver: changed the speed multiplier to left trigger and changed by toggle
        //speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());

        // temporary setting just for testing purpose
        // Intake bindings
        Trigger horizontalSlideSpeedVariationTrigger = new Trigger(() -> isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        horizontalSlideSpeedVariationTrigger.whenActive(() -> intakeSubsystem.changeHorizontalSlideSpeedMultiplier());

        Trigger runActiveIntake = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A));
        runActiveIntake.whileActiveContinuous(()-> intakeSubsystem.runActiveIntakeServo());
        runActiveIntake.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger turnIntakeWrist = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B));
        turnIntakeWrist.whileActiveContinuous(()-> intakeSubsystem.servoDownPosition());
        turnIntakeWrist.whenInactive(()-> intakeSubsystem.servoUpPosition());

        RunCommand defaultHorizontalSlideCommand = new RunCommand(() -> intakeSubsystem.runHorizontalSlides(operatorGamepad.getLeftY()));
        defaultHorizontalSlideCommand.addRequirements(intakeSubsystem);

        intakeSubsystem.setDefaultCommand(defaultHorizontalSlideCommand);



    }

    public void configureAutoSetting(){
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();


    }

    public void run() {
        CommandScheduler.getInstance().run();
        intakeSubsystem.updateHorizontalSlideDistance();
        

        opMode.telemetry.addData("Speed Multiplier",driveSubsystem.speedMultiplier);
        opMode.telemetry.addData("Y axis:", operatorGamepad.getLeftY());
        opMode.telemetry.addData("Is fieldcentric?",driveSubsystem.fieldCentric);
        opMode.telemetry.addData("value: ", intakeSubsystem.getDistanceTraveled());
        opMode.telemetry.update();
    }

    public  boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}
