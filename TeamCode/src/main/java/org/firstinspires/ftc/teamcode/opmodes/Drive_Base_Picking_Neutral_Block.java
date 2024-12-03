package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="Drive_base_Picking_Neutral_Block", group = "Real")
public class Drive_Base_Picking_Neutral_Block extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();


        int allianceNumber = 1;//blue = 1, red = -1

        Pose2d initialPose = new Pose2d(-3 * Constants.INCH_TO_TILE * allianceNumber, 1.5 * Constants.INCH_TO_TILE * allianceNumber, Math.toRadians(90 * allianceNumber));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Pose2d updatedPose = new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble());

        TrajectoryActionBuilder getNeutral = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber,2 * Constants.INCH_TO_TILE * allianceNumber),Math.toRadians(270 * allianceNumber))
                .strafeTo(new Vector2d(-2.9 * Constants.INCH_TO_TILE * allianceNumber,2.5 * Constants.INCH_TO_TILE * allianceNumber));

        TrajectoryActionBuilder parkObzone = drive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-2.5 * Constants.INCH_TO_TILE * allianceNumber,-2.5 * Constants.INCH_TO_TILE * allianceNumber));//drive straight into ob zone




        waitForStart();

        Action chosenGettingBlockAction = getNeutral.build();
        Action chosenParkingAction = parkObzone.build();

        Actions.runBlocking(
                new SequentialAction(
                        chosenGettingBlockAction,
                        chosenParkingAction

                ));//execute the planned action

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
        }
    }
}
