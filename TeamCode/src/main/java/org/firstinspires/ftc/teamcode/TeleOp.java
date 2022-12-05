package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.karrmedia.ftchotpatch.Supervised;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.i18nformatter.qual.I18nValidFormat;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveCancelable;

//@Supervised(name="?TeleOp", group="Iterative Opmode", autonomous=false, variations={"Red", "Blue"})
@Supervised(name="TeleOp", group="!CompTeleOp", autonomous=false)
public class TeleOp extends SupervisedOpMode {
    RobotA robot;
    GamepadButton liftButton;
    GamepadButton liftCloseButton;
    ToggleButtonReader clawButton;
    ToggleButtonReader fineControls;

    // Code that runs when the INIT button is pressed (mandatory)
    public void init() {
        robot = new RobotA(hardwareMap);
        //robot.slide.winch.setRunMode(Motor.RunMode.RawPower);

        //liftButton = new GamepadButton(gamepad1, GamepadKeys.Button.A);
        //liftCloseButton = new GamepadButton(gamepad1, GamepadKeys.Button.B);
        //liftButton = new GamepadButton(gamepad1, GamepadKeys.Button.X);
        clawButton = new ToggleButtonReader(gamepad1, GamepadKeys.Button.X);
        fineControls = new ToggleButtonReader(gamepad1, GamepadKeys.Button.Y);
    }

    public void start() {

    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {
        gamepad1.readButtons();
        clawButton.readValue();
        fineControls.readValue();


        if (fineControls.getState()) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.getLeftY() / 3.0,
                            -gamepad1.getLeftX() / 3.0,
                            ((-gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) +
                                    gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 5.0) / 9.0
                    )
            );
        }
        else {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.getLeftY(),
                            -gamepad1.getLeftX(),
                            ((-gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) +
                                    gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 5.0) / 3.0
                    )
            );
        }

        //robot.slide.winch.set(-gamepad1.right_stick_y);
        //if (gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
        //    robot.slide.setCurrentWinchTarget((int) (robot.slide.currentWinchTarget - (gamepad1.getRightY() * 10)));
        //}
        if (gamepad1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.slide.claw.rotateByAngle(((gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 0.0 : 1.0) + ((gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 0.0 : -1.0))) / 10.0);
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.slide.winch.resetEncoder();
            robot.slide.currentWinchTarget = 0;
        }

        if (gamepad1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            if (gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robot.slide.disableAutomaticWinch();
            }

            robot.slide.winch.set(-gamepad1.getRightY());
        }
        if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            robot.slide.enableAutomaticWinch();
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
            robot.slide.goToNextSlidePos();
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
            robot.slide.setCurrentWinchTarget(robot.slide.SLIDE_POS_BOTTOM);
        }

        if (clawButton.stateJustChanged()) {
            if (clawButton.getState()) {
                robot.slide.claw.turnToAngle(robot.slide.CLAW_POS_CLOSED);
            }
            else {
                robot.slide.claw.turnToAngle(robot.slide.CLAW_POS_OPEN);
            }
        }

        telemetry.addData("Claw Position", robot.slide.claw.getPosition());
        telemetry.addData("Claw Angle", robot.slide.claw.getAngle());
        telemetry.addData("Winch Level", robot.slide.winch.getCurrentPosition());
        telemetry.addData("Winch Target", robot.slide.currentWinchTarget);



        /*if (gamepad1.wasJustPressed(GamepadKeys.Button.X)) {
            robot.slide.claw.setPosition(robot.slide.CLAW_POS_CLOSED);
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.slide.claw.setPosition(robot.slide.CLAW_POS_OPEN);
        }*/

        /*int factor = 7;
        claw.rotateByAngle(gamepad1.right_stick_y * 2);

        telemetry.addData("ClawAngle", claw.getAngle());*/

        robot.update();

        telemetry.update();
    }

    // Code that runs when the OpMode is stopped (optional)
    public void stop() {

    }

    // Code that runs after this OpMode is dynamically updated
    public void hotpatch() {

    }

    public int mySpecialNotDuplicatedFunction() { return 5; }
}