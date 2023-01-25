package org.firstinspires.ftc.teamcode.AMLTresAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AMLOneAutos.BetterOdomPark;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBitMap;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "lowJunctionAuto", group = "Autonomous")

public class lowJunctionAuto extends LinearOpMode{
    enum State {

        IDLE,
        WAIT,
        GO_TO_WALL,
        GO_TO_STACK,
        TURN_TO_ALIGN_STACK,
        GO_FORWARD_PICK_UP_CONE,
        SCORE_LOW,
        GO_FORWARD_PICK_UP_CONE2,
        SCORE_LOW2,
        PARK
    }

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d posEstimate;

    public static double goToWallX = 0;
    public static double goToWallY = -40;
    public static double goToWallAngle = 0;

    public static double goToStackX = 12;
    public static double goToStackY = -40;
    public static double goToStackAngle = 0;

    public static double turnToAlignX = 31;
    public static double turnToAlignY = -40;
    public static double turnToAlignAngle = Math.toRadians(-258);

    public static double pickUpConeX = 31;
    public static double pickUpConeY = -40;
    public static double pickUpConeAngle = Math.toRadians(-258);

    public static double scoreLowX = 31;
    public static double scoreLowY = -40;
    public static double scoreLowAngle = Math.toRadians(-258);

    public static double pickUpCone2X = 31;
    public static double pickUpCone2Y = -40;
    public static double pickUpCone2Angle = Math.toRadians(-258);



    public static double startWait = 0;

    lowJunctionAuto.State currentState = lowJunctionAuto.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VuforiaBitMap vuforia = new VuforiaBitMap(this);

        int pos = vuforia.LeftPostionVision();
        telemetry.addData("Pos: ", pos);
        telemetry.update();

        drive.setPoseEstimate(startPose);

        ElapsedTime waitTimer = new ElapsedTime();

        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        Trajectory goToWall = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(goToWallX, goToWallY,goToWallAngle ))
                .build();


        Trajectory goToStack = drive.trajectoryBuilder(goToWall.end())
                .lineToLinearHeading(new Pose2d(goToStackX, goToStackY, goToStackAngle))
                .build();


        Trajectory turnToAlign = drive.trajectoryBuilder(goToStack.end())
                .lineToLinearHeading(new Pose2d(turnToAlignX, turnToAlignY, turnToAlignAngle))
                .build();

        Trajectory pickUpCone = drive.trajectoryBuilder(turnToAlign.end())
                .lineToLinearHeading(new Pose2d(pickUpConeX, pickUpConeY, pickUpConeAngle))
                .build();

        Trajectory scoreLow = drive.trajectoryBuilder(pickUpCone.end())
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

        Trajectory pickUpCone2 = drive.trajectoryBuilder(scoreLow.end())
                .lineToLinearHeading(new Pose2d(scoreLowX, scoreLowY, scoreLowAngle))
                .build();

/*
        Trajectory pos_dos = drive.trajectoryBuilder(align2.end())
                .lineToLinearHeading(new Pose2d(align2x, (align2y+20), align2angle))
                .build();

        Trajectory pos_tres = drive.trajectoryBuilder(align2.end())
                .lineToLinearHeading(new Pose2d(align2x, (align2y+(23*2)), align2angle))
                .build();
*/

        currentState = lowJunctionAuto.State.WAIT;



        while (opModeIsActive() && !isStopRequested()) {

            posEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:

                    if (waitTimer.seconds() > startWait) {

                        currentState = lowJunctionAuto.State.GO_TO_WALL;
                    }

                    break;

                case GO_TO_WALL:

                    drive.followTrajectory(goToWall);
                    currentState = State.GO_TO_STACK;


                    break;

                case GO_TO_STACK:
                    drive.followTrajectory(goToStack);
                    currentState = State.TURN_TO_ALIGN_STACK;


                    break;

                case TURN_TO_ALIGN_STACK:
                    drive.followTrajectory(turnToAlign);
                    currentState = State.GO_FORWARD_PICK_UP_CONE;


                    break;

                case GO_FORWARD_PICK_UP_CONE:
                    drive.followTrajectory(pickUpCone);
                    currentState = State.SCORE_LOW;


                    break;

                case SCORE_LOW:
                    drive.followTrajectory(scoreLow);
                    currentState = State.GO_FORWARD_PICK_UP_CONE2;


                    break;

                case GO_FORWARD_PICK_UP_CONE2:
                    drive.followTrajectory(pickUpCone2);
                    currentState = State.SCORE_LOW;


                    break;

                case SCORE_LOW2:
                    drive.followTrajectory(scoreLow);
                    currentState = State.PARK;


                    break;
/*
                case PARK:
                    if      (pos==1){
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else if (pos==2){
                        drive.followTrajectory(pos_dos);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    else{
                        drive.followTrajectory(pos_tres);
                        currentState = lowJunctionAuto.State.IDLE;
                    }
                    break;
*/
                case IDLE:

                    break;
            }
        }
    }
}
