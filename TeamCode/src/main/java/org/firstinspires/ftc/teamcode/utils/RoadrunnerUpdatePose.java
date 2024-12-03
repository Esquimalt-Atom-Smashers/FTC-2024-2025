package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElbowSubsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


public  class RoadrunnerUpdatePose extends CommandBase {
    private MecanumDrive roadrunnerDrive;

    public RoadrunnerUpdatePose(MecanumDrive roadrunnerDrive) {
        this.roadrunnerDrive = roadrunnerDrive;
    }

    @Override
    public void execute() {
        roadrunnerDrive.updatePoseEstimate();

        telemetry.addData("x", roadrunnerDrive.pose.position.x);
        telemetry.addData("y", roadrunnerDrive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(roadrunnerDrive.pose.heading.toDouble()));
        telemetry.update();


    }

    public Pose2d returnUpdatedPose(){
        return new Pose2d(roadrunnerDrive.pose.position.x,roadrunnerDrive.pose.position.y,roadrunnerDrive.pose.heading.toDouble());
    }
}