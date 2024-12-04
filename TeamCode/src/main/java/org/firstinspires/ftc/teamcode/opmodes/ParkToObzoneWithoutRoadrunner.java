package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Autonomous", group = "Real")
public class ParkToObzoneWithoutRoadrunner extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

    }

    @Override
    public void loop() {
        robot.run();
        robot.autoModeDrive(1,0,0,3);
    }
}