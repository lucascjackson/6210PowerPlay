package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.BNO055Wrapper;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.OverflowEncoder;
import org.firstinspires.ftc.teamcode.util.RawEncoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

final class DriveView {
    public final List<DcMotorEx> leftMotors, rightMotors;
    public final List<DcMotorEx> motors;

    // invariant: (leftEncs.isEmpty() && rightEncs.isEmpty()) ||
    //                  (parEncs.isEmpty() && perpEncs.isEmpty())
    public final List<RawEncoder> leftEncs, rightEncs, parEncs, perpEncs;
    public final List<RawEncoder> forwardEncs;

    public final BNO055Wrapper imu;

    public final VoltageSensor voltageSensor;

    public final String type;

    private static RawEncoder unwrap(Encoder e) {
        if (e instanceof OverflowEncoder) {
            return ((OverflowEncoder) e).encoder;
        } else {
            return (RawEncoder) e;
        }
    }

    public DriveView(Object d) {
        final Localizer localizer;
        if (d instanceof MecanumDrive) {
            type = "mecanum";

            MecanumDrive md = (MecanumDrive) d;
            leftMotors = Arrays.asList(md.leftFront, md.leftRear);
            rightMotors = Arrays.asList(md.rightFront, md.rightRear);
            imu = md.imu;
            voltageSensor = md.voltageSensor;

            localizer = md.localizer;
        } else if (d instanceof TankDrive) {
            type = "tank";

            TankDrive td = (TankDrive) d;
            leftMotors = Collections.unmodifiableList(td.leftMotors);
            rightMotors = Collections.unmodifiableList(td.rightMotors);
            imu = td.imu;
            voltageSensor = td.voltageSensor;

            localizer = td.localizer;
        } else {
            throw new AssertionError();
        }

        if (localizer instanceof TwoDeadWheelLocalizer) {
            TwoDeadWheelLocalizer l2 = (TwoDeadWheelLocalizer) localizer;
            parEncs = Collections.singletonList(unwrap(l2.par));
            perpEncs = Collections.singletonList(unwrap(l2.perp));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();
        } else if (localizer instanceof ThreeDeadWheelLocalizer) {
            ThreeDeadWheelLocalizer l3 = (ThreeDeadWheelLocalizer) localizer;
            parEncs = Arrays.asList(unwrap(l3.par0), unwrap(l3.par1));
            perpEncs = Collections.singletonList(unwrap(l3.perp));
            leftEncs = Collections.emptyList();
            rightEncs = Collections.emptyList();
        } else if (localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = Arrays.asList(unwrap(dl.leftFront), unwrap(dl.leftRear));
            rightEncs = Arrays.asList(unwrap(dl.rightFront), unwrap(dl.rightRear));
        } else if (localizer instanceof TankDrive.DriveLocalizer) {
            TankDrive.DriveLocalizer dl = (TankDrive.DriveLocalizer) localizer;
            parEncs = Collections.emptyList();
            perpEncs = Collections.emptyList();
            leftEncs = new ArrayList<>();
            for (Encoder e : dl.leftEncs) {
                leftEncs.add(unwrap(e));
            }
            rightEncs = new ArrayList<>();
            for (Encoder e : dl.rightEncs) {
                rightEncs.add(unwrap(e));
            }
        } else {
            throw new AssertionError();
        }

        List<DcMotorEx> motors = new ArrayList<>();
        motors.addAll(leftMotors);
        motors.addAll(rightMotors);
        this.motors = Collections.unmodifiableList(motors);

        List<RawEncoder> forwardEncs = new ArrayList<>();
        forwardEncs.addAll(leftEncs);
        forwardEncs.addAll(rightEncs);
        forwardEncs.addAll(parEncs);
        this.forwardEncs = Collections.unmodifiableList(forwardEncs);

        List<RawEncoder> allEncs = new ArrayList<>();
        allEncs.addAll(forwardEncs);
        allEncs.addAll(perpEncs);

        DcMotorController c1 = allEncs.get(0).getController();
        for (Encoder e : allEncs) {
            DcMotorController c2 = e.getController();
            if (c1 != c2) {
                throw new IllegalArgumentException("all encoders must be attached to the same hub");
            }
        }
    }
}