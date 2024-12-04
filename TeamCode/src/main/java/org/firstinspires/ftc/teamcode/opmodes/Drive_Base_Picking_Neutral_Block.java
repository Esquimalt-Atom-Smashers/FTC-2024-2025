package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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

        int allianceNumber = 1;//blue = 1, red = -1

        Pose2d initialPose = new Pose2d(-3 * Constants.INCH_TO_TILE * allianceNumber, 1.5 * Constants.INCH_TO_TILE * allianceNumber, Math.toRadians(90 * allianceNumber));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Pose2d updatedPose = new Pose2d(drive.pose.position.x,drive.pose.position.y,drive.pose.heading.toDouble());

        TrajectoryActionBuilder getNeutralBlock1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber, 1.5 * Constants.INCH_TO_TILE * allianceNumber))
                .splineToConstantHeading(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber,2 * Constants.INCH_TO_TILE * allianceNumber),Math.toRadians(0))
                .strafeTo(new Vector2d(-2.9 * Constants.INCH_TO_TILE * allianceNumber,2.5 * Constants.INCH_TO_TILE * allianceNumber));
        Action getBlock1 = getNeutralBlock1.build();

        TrajectoryActionBuilder getNeutralBlock2 = drive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber, 1.5 * Constants.INCH_TO_TILE * allianceNumber))
                .splineToConstantHeading(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber,2.5 * Constants.INCH_TO_TILE * allianceNumber),Math.toRadians(0))
                .strafeTo(new Vector2d(-2.9 * Constants.INCH_TO_TILE * allianceNumber,2.5 * Constants.INCH_TO_TILE * allianceNumber));
        Action getBlock2 = getNeutralBlock2.build();

        TrajectoryActionBuilder getNeutralBlock3 = drive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber, 1.5 * Constants.INCH_TO_TILE * allianceNumber))
                .splineToConstantHeading(new Vector2d(-0.5 * Constants.INCH_TO_TILE * allianceNumber,2.9 * Constants.INCH_TO_TILE * allianceNumber),Math.toRadians(0))
                .strafeTo(new Vector2d(-2.9 * Constants.INCH_TO_TILE * allianceNumber,2.5 * Constants.INCH_TO_TILE * allianceNumber));
        Action getBlock3 = getNeutralBlock3.build();

        TrajectoryActionBuilder parkObzone = drive.actionBuilder(updatedPose)
                .strafeTo(new Vector2d(-2.5 * Constants.INCH_TO_TILE * allianceNumber,-2.5 * Constants.INCH_TO_TILE * allianceNumber));//drive straight into ob zone

        waitForStart();
        Action chosenParkingAction = parkObzone.build();

        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
            drive.updatePoseEstimate();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                getBlock1,
                                getBlock2,
                                getBlock3,
                                chosenParkingAction
                        ),
                        robot.roadrunnerUpdatePose()
                        )
                );//execute the planned action
        }
    }
}
