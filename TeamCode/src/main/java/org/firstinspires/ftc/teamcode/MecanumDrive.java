package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Twist2;
import com.acmerobotics.roadrunner.Twist2IncrementDual;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LynxFirmwareVersion;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

@Config
public final class MecanumDrive {
    public static double FORWARD_IN_PER_TICK = 0;
    public static double LATERAL_IN_PER_TICK = 1;
    public static double TRACK_WIDTH_TICKS = 0;

    public final MecanumKinematics kinematics;

    public final DcMotorEx leftFront, leftRear, rightRear, rightFront;

    public final VoltageSensor voltageSensor;

    public final BNO055Wrapper imu;

    public final Localizer localizer = new DriveLocalizer();
    public Transform2 txRobotWorld = new Transform2(new Vector2(0, 0), Rotation2.exp(0));

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftRear, rightRear, rightFront;

        private int lastLeftFrontPos, lastLeftRearPos, lastRightRearPos, lastRightFrontPos;

        public DriveLocalizer() {
            leftFront = new RawEncoder(MecanumDrive.this.leftFront);
            leftRear = new RawEncoder(MecanumDrive.this.leftRear);
            rightRear = new RawEncoder(MecanumDrive.this.rightRear);
            rightFront = new RawEncoder(MecanumDrive.this.rightFront);

            lastLeftFrontPos = leftFront.getPositionAndVelocity().position;
            lastLeftRearPos = leftRear.getPositionAndVelocity().position;
            lastRightRearPos = rightRear.getPositionAndVelocity().position;
            lastRightFrontPos = rightFront.getPositionAndVelocity().position;
        }

        @Override
        public Twist2IncrementDual<Time> updateAndGetIncr() {
            Encoder.PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            Encoder.PositionVelocityPair leftRearPosVel = leftRear.getPositionAndVelocity();
            Encoder.PositionVelocityPair rightRearPosVel = rightRear.getPositionAndVelocity();
            Encoder.PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            MecanumKinematics.WheelIncrements<Time> incrs = new MecanumKinematics.WheelIncrements<>(
                    new DualNum<>(new double[] {
                            FORWARD_IN_PER_TICK * (leftFrontPosVel.position - lastLeftFrontPos),
                            FORWARD_IN_PER_TICK * leftFrontPosVel.velocity,
                    }),
                    new DualNum<>(new double[] {
                            FORWARD_IN_PER_TICK * (leftRearPosVel.position - lastLeftRearPos),
                            FORWARD_IN_PER_TICK * leftRearPosVel.velocity,
                    }),
                    new DualNum<>(new double[] {
                            FORWARD_IN_PER_TICK * (rightRearPosVel.position - lastRightRearPos),
                            FORWARD_IN_PER_TICK * rightRearPosVel.velocity,
                    }),
                    new DualNum<>(new double[] {
                            FORWARD_IN_PER_TICK * (rightFrontPosVel.position - lastRightFrontPos),
                            FORWARD_IN_PER_TICK * rightFrontPosVel.velocity,
                    })
            );

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftRearPos = leftRearPosVel.position;
            lastRightRearPos = rightRearPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            return kinematics.forward(incrs);
        }
    }

    public MecanumDrive(HardwareMap hardwareMap) {
        kinematics = new MecanumKinematics(
                FORWARD_IN_PER_TICK * TRACK_WIDTH_TICKS,
                FORWARD_IN_PER_TICK / LATERAL_IN_PER_TICK);

        LynxFirmwareVersion.throwIfAnyModulesBelowVersion(hardwareMap,
                new LynxFirmwareVersion(1, 8, 2));

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU baseImu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        baseImu.initialize(parameters);
        imu = new BNO055Wrapper(baseImu);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public Twist2 updatePoseEstimateAndGetActualVel() {
        Twist2IncrementDual<Time> incr = localizer.updateAndGetIncr();
        txRobotWorld = txRobotWorld.plus(incr.value());
        return incr.velocity().value();
    }
}