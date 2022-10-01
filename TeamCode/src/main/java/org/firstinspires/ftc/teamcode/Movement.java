package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Movement {

    HardwareMap robot;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private boolean halfspeed = false;

    public Movement(HardwareMap hardwareMap) {
        this.robot = hardwareMap;

        FL = robot.get(DcMotor.class, "FL");
        FR = robot.get(DcMotor.class, "FR");
        BL = robot.get(DcMotor.class, "BL");
        BR = robot.get(DcMotor.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
    }

    public double[] holonomicDrive(double leftX, double leftY, double rightX) {
        double[] motorpowers = new double[4];
        motorpowers[0] = leftY + leftX + rightX;
        motorpowers[1] = leftY - leftX - rightX;
        motorpowers[2] = leftY - leftX + rightX;
        motorpowers[3] = leftY + leftX - rightX;
        return motorpowers;
    }

    public void setPowers(double[] motorPower) {

        if (halfspeed) {

            for(int i = 0; i<4; i++) {
                motorPower[i] /= 2;
            }
        }

        FR.setPower(motorPower[0]);
        FL.setPower(motorPower[1]);
        BR.setPower(motorPower[2]);
        BL.setPower(motorPower[3]);
    }

    public void setHalfspeed() {

        halfspeed = !halfspeed;

    }


}
