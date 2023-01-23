package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Lift Test")
public class LiftTest extends LinearOpMode {
    Manipulators manip;
    private DcMotor lift2;
    public DigitalChannel liftSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        lift2 = hardwareMap.dcMotor.get("lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftSensor = hardwareMap.get(DigitalChannel.class, "liftSensor");

        telemetry.setAutoClear(false);
        Telemetry.Item liftPosition = telemetry.addData("Lift Position", lift2.getCurrentPosition());


        while(!isStarted()){
            liftPosition.setValue(lift2.getCurrentPosition());
            telemetry.update();
        }



        int liftTarget = 0;
        double liftSpeed = 0;
        String liftDirection = "up";

        while (opModeIsActive()) {

            if (gamepad1.b){ // lift UP
                liftTarget = 4300;
                liftSpeed = 0.98;
                liftDirection = "up";

                lift2.setPower(liftSpeed);
                lift2.setTargetPosition(liftTarget);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad1.a){ // lift DOWN
                liftTarget = 0;
                liftSpeed = -0.98;
                liftDirection = "down";

                lift2.setPower(liftSpeed);
                lift2.setTargetPosition(liftTarget);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if ( liftDirection == "down" && !liftSensor.getState()){
                liftSpeed = 0;
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }



            idle();
            if( lift2.isBusy() ){
                liftPosition.setValue(lift2.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}