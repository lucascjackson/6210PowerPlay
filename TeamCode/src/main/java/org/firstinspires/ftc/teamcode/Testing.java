package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Testing", group="TeleOp")
public class Testing extends OpMode {

    Movement move;
    Manipulators manip;
    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    double[] motorPower = {0, 0, 0, 0};

    double FRM = .5;
    double FLM = .5;
    double BRM = .5;
    double BLM = .5;


    double leftY;
    double leftX;
    double rightX;

    public void init() {
        move = new Movement(hardwareMap);
        manip = new Manipulators(hardwareMap);

        //positions = manip.setPositions();
        telemetry.addData("init", "completed");
        telemetry.update();
    }

    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)) {
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
    }

    @Override
    public void loop() {

        if (isPressed("y2", gamepad2.y)) {
            FLM += 0.1;
        }
        if (isPressed("b2", gamepad2.b)) {
            FRM += 0.1;
        }
        if (isPressed("x2", gamepad2.x)) {
            BLM += 0.1;
        }
        if (isPressed("a2", gamepad2.a)) {
            BRM += 0.1;
        }

        if (isPressed("dpad_up2", gamepad2.dpad_up)) {
            FLM -= 0.1;
        }
        if (isPressed("dpad_right2", gamepad2.dpad_right)) {
            FRM -= 0.1;
        }
        if (isPressed("dpad_left2", gamepad2.dpad_left)) {
            BLM -= 0.1;
        }
        if (isPressed("dpad_down2", gamepad2.dpad_down)) {
            BRM -= 0.1;
        }



        if (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y;
            leftX = -1 * (gamepad1.left_stick_x);
            rightX = (-gamepad1.right_stick_x);

            motorPower = move.holonomicDrive(leftX, leftY, rightX);

        } else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }

        if (isPressed("dpad_up1", gamepad1.dpad_up)) {
            manip.moveLiftTo(-1000);
        }

        if (isPressed("dpad_down1", gamepad1.dpad_down)) {
            manip.returnLiftToDefault();
        }

        move.setPowers(motorPower[0]*FRM, motorPower[1]*FLM, motorPower[2]*BRM, motorPower[3]*BLM);

        telemetry.addData("FR power: ", FRM);
        telemetry.addData("FL power: ", FLM);
        telemetry.addData("BR power: ", BRM);
        telemetry.addData("BL power: ", BLM);
        telemetry.addData("Blue: ", manip.colorBlue());
        telemetry.addData("Red: ", manip.colorRed());
        telemetry.addData("lift1:", manip.lift1.getCurrentPosition());
        telemetry.addData("lift2:", manip.lift2.getCurrentPosition());


        telemetry.update();
    }
}
