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
    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();
    double[] motorPower = {0, 0, 0, 0};

    double leftYM = .5;
    double leftXM = .5;
    double rightXM = .5;



    public void init() {
        move = new Movement(hardwareMap);


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
        double leftY = 0;
        double leftX = 0;
        double rightX = 0;


        if (isPressed("y2", gamepad2.y)) {
            move.FLMulti(.1);
        }
        if (isPressed("b2", gamepad2.b)) {
            move.FRMulti(.1);
        }
        if (isPressed("x2", gamepad2.x)) {
            move.BLMulti(.1);
        }
        if (isPressed("a2", gamepad2.a)) {
            move.BRMulti(.1);
        }

        if (isPressed("dpad_up2", gamepad2.dpad_up)) {
            move.FLMulti(-.1);
        }
        if (isPressed("dpad_right2", gamepad2.dpad_right)) {
            move.FRMulti(-.1);
        }
        if (isPressed("dpad_left2", gamepad2.dpad_left)) {
            move.BLMulti(-.1);
        }
        if (isPressed("dpad_down2", gamepad2.dpad_down)) {
            move.BRMulti(-.1);
        }



        if (Math.abs(gamepad1.left_stick_y) > 0.1 ||
                Math.abs(gamepad1.left_stick_x) > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y;
            leftX = -1 * (gamepad1.left_stick_x);
            rightX = (-gamepad1.right_stick_x);

            motorPower = move.holonomicDrive(leftX*leftXM, leftY*leftYM, rightX*rightXM);

        } else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }

        move.setPowers(motorPower);

        telemetry.addData("FL: ", .5);
        telemetry.addData("StickY: ", gamepad2.right_stick_y);
        telemetry.update();
    }
}
