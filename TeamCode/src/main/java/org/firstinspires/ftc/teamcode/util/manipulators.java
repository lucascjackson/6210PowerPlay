package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class manipulators {

    private HardwareMap robot = null;
    //Lowest and Highest Encoder Positions
    public int lowest = 0;
    public int highest = 0;
    //intake
    public DcMotor intake1 = null;
    public DcMotor intake2 = null;

    //lift
    public DcMotor lift1 = null;

    //outtake
    public Servo outtake = null;

    //Hardware Map
    public manipulators(HardwareMap robot) {
        this.robot = robot;
        //intake mapping
        intake1 = robot.get(DcMotor.class, "intake1");
        intake2 = robot.get(DcMotor.class, "intake2");
        //lift mapping
        lift1 = robot.get(DcMotor.class, "lift1");

        //outtake mapping
        outtake = robot.get(Servo.class, "outtake");

        //lift Encoders
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkPosition() {
        if (lift1.getCurrentPosition() < lowest) {
            lowest = lift1.getCurrentPosition();
        }
        if (lift1.getCurrentPosition() > highest) {
            highest = lift1.getCurrentPosition();
        }
    }

}
