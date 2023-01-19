package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.karrmedia.ftchotpatch.SupervisedOpMode;
import com.karrmedia.ftchotpatch.Supervised;

import org.firstinspires.ftc.teamcode.wpi.first.math.filter.LinearFilter;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

//@Supervised(name="?TeleOp", group="Iterative Opmode", autonomous=false, variations={"Red", "Blue"})
@Config
@Supervised(name="TeleOp", group="!CompTeleOp", autonomous=false, linear=false)
public class TeleOp extends SupervisedOpMode {
    RobotA robot;
    GamepadButton liftButton;
    GamepadButton liftCloseButton;
    GamepadButton turnAroundButton;
    ToggleButtonReader clawButton;
    ToggleButtonReader fineControls;
    ToggleButtonReader fieldCentric;

    // Code that runs when the INIT button is pressed (mandatory)
    public void init() {
        robot = TeamConf.getRobot(hardwareMap);

        robot.slide.claw.turnToAngle(robot.slide.claw.getAngle() + 10);
        robot.slide.claw.turnToAngle(robot.slide.claw.getAngle() - 10);

        //robot.slide.winch.setRunMode(Motor.RunMode.RawPower);

        //liftButton = new GamepadButton(gamepad1, GamepadKeys.Button.A);
        //liftCloseButton = new GamepadButton(gamepad1, GamepadKeys.Button.B);
        //liftButton = new GamepadButton(gamepad1, GamepadKeys.Button.X);
        turnAroundButton = new GamepadButton(gamepad1, GamepadKeys.Button.LEFT_BUMPER);
        clawButton = new ToggleButtonReader(gamepad1, GamepadKeys.Button.X);
        fineControls = new ToggleButtonReader(gamepad1, GamepadKeys.Button.Y);
        fieldCentric = new ToggleButtonReader(gamepad1, GamepadKeys.Button.RIGHT_BUMPER);
    }

    public void start() {
        elapsedRuntime.reset();

        while (super.opModeIsActive()) {
            loop();
        }
    }

    public static double DRIVE_SLOW_MULTIPLIER = 20.0;
    public static double DRIVE_SLOW_TURN_MULTIPLIER = 3.0;
    public double triggerValue(double unadjusted) {
        return (unadjusted - 0.05) / (1.0 - 0.05);
    }

    // Code that runs repeatedly after the PLAY button is pressed (optional)
    public void loop() {
        //count();

        gamepad1.readButtons();

        clawButton.readValue();
        fineControls.readValue();
        fieldCentric.readValue();

        //robot.drive.cancelFollowing();
        robot.drive.updatePoseEstimate();

        double mult = fineControls.getState() ? DRIVE_SLOW_MULTIPLIER : 1.0;
        double turnMult = fineControls.getState() ? DRIVE_SLOW_TURN_MULTIPLIER : 1.0;
        if (!fieldCentric.getState()) {
            robot.manualDrive.driveRobotCentric(gamepad1.getLeftY() * mult, gamepad1.getLeftX() * mult,
                    (-triggerValue(gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) +
                            triggerValue(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) * turnMult,
                    true);
        }
        else {
            robot.manualDrive.driveFieldCentric(gamepad1.getLeftY() * mult, gamepad1.getLeftX() * mult,
                    (-triggerValue(gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) +
                    triggerValue(gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))) * mult,
                            robot.drive.getExternalHeadingVelocity(), true);
        }


        /*}
        else {
            if (robot.drive.isBusy()) {
                robot.drive.update();
            }
        }*/

        if (turnAroundButton.get()) {
            robot.drive.turnAsync(Math.PI, true);
        }

        //robot.slide.winch.set(-gamepad1.right_stick_y);
        //if (gamepad1.isDown(GamepadKeys.Button.DPAD_UP)) {
        //    robot.slide.setCurrentWinchTarget((int) (robot.slide.currentWinchTarget - (gamepad1.getRightY() * 10)));
        //}
        if (gamepad1.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            //robot.slide.claw.rotateByAngle(((gamepad1.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 0.0 : 1.0) + ((gamepad1.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 0.0 : -1.0))) / 10.0);
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.slide.winch.resetEncoder();
            robot.slide.currentWinchTarget = 0;
        }

        robot.slide.winch.set(-gamepad1.getRightY());

        /*if (gamepad1.isDown(GamepadKeys.Button.DPAD_LEFT)) {
            if (gamepad1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                robot.slide.disableAutomaticWinch();
            }

            robot.slide.winch.set(-gamepad1.getRightY());
        }
        if (gamepad1.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
            robot.slide.enableAutomaticWinch();
        }*/

        if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
            robot.slide.goToNextSlidePos();
        }

        if (gamepad1.wasJustPressed(GamepadKeys.Button.B)) {
            robot.slide.setCurrentWinchTarget(robot.slide.SLIDE_POS_BOTTOM);
        }

        if (clawButton.stateJustChanged()) {
            if (clawButton.getState()) {
                //robot.slide.claw.turnToAngle(robot.slide.CLAW_POS_CLOSED);
                robot.slide.setClawTarget(robot.slide.CLAW_POS_CLOSED);
            }
            else {
                //robot.slide.claw.turnToAngle(robot.slide.CLAW_POS_OPEN);
                robot.slide.setClawTarget(robot.slide.CLAW_POS_OPEN);
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



    long lastSec = 0;
    long called = 0;
    List<Double> calledCounts = new ArrayList<Double>();

    void count() {
        long now = elapsedRuntime.now(TimeUnit.SECONDS);
        called++;

        if (now > lastSec) {
            calledCounts.add(1.0 / called);
            called = 0;
            lastSec = now;

            //telemetry.clear();
            //telemetry.addData("Loops per sec", calledCounts);
            //telemetry.update();
        }
    }
}