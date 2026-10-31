package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.depollinator.Depollinator;
import org.firstinspires.ftc.teamcode.depollinator.DepollinatorCommands;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.ChassisCommands;
import org.firstinspires.ftc.teamcode.chassis.DriveConstants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.vision.Limelight;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer extends Robot {
    public static Pose ROBOT_POSE = null;
    private final List<SubsystemIF> subsystems = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private final List<LynxModule> hubs;
    private final Chassis chassis;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Intake intake;

    private final Depollinator depollinator;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private GamepadEx gamepad1;
    private GamepadEx gamepad2;
    private final Gamepad rawGamepad1;
    private final Gamepad rawGamepad2;
    private boolean teleop = false;
    private boolean wasShooterReady = false;
    private boolean wasAimLocked = false;

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        reset();

        this.hardwareMap = hardwareMap;
        this.telemetry = new JoinedTelemetry(
                telemetry,
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
        this.rawGamepad1 = gamepad1;
        this.rawGamepad2 = gamepad2;
        PanelsDrawing.init();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        chassis = new Chassis(this.telemetry, hardwareMap);
        limelight = new Limelight(this.telemetry, hardwareMap, chassis);
        shooter = new Shooter(this.telemetry, hardwareMap);
        intake = new Intake(this.telemetry, hardwareMap);
        depollinator = new Depollinator(this.telemetry, hardwareMap);

        shooter.setDistanceSupplier(() -> limelight.getDistance(chassis.getPose()));

        subsystems.addAll(List.of(chassis, limelight, intake, shooter, depollinator));

        for(Subsystem s : subsystems) {
            register(s);
        }
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
    }

    public void autonomousInit() {
        for(SubsystemIF s : subsystems) {
            s.autonomousInit();
        }
    }

    public void teleopInit() {
        for(SubsystemIF s : subsystems) {
            s.teleopInit();
        }

        teleop = true;

        chassis.setDefaultCommand(ChassisCommands.createTeleopDriveCommand(
                chassis,
                limelight,
                () -> gamepad1.getLeftY() * driveScale(),
                () -> -gamepad1.getLeftX() * driveScale(),
                () -> -gamepad1.getRightX() * turnScale(),
                this::aimHeld));

        shooter.setFeedPermission(() -> !aimHeld() || aimLocked());

        gamepad1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(chassis::resetHeading);

        gamepad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(ShooterCommands.enableShooter(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(ShooterCommands.disableShooter(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(ShooterCommands.shoot(shooter))
                .whenReleased(ShooterCommands.disableShooterServo(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(ShooterCommands.trimUp(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(ShooterCommands.trimDown(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(ShooterCommands.resetTrim(shooter));

        gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(IntakeCommands.intake(intake))
                .whenReleased(IntakeCommands.stopIntake(intake));
        gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(DepollinatorCommands.depollinate(depollinator))
                .whenReleased(DepollinatorCommands.restDepollinator(depollinator));
        gamepad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(IntakeCommands.outtake(intake))
                .whenReleased(IntakeCommands.stopIntake(intake));
    }

    private boolean aimHeld() {
        return gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3;
    }

    private boolean slowHeld() {
        return gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3;
    }

    private double driveScale() {
        return slowHeld() ? DriveConstants.SLOW_MODE_DRIVE_SCALE : 1.0;
    }

    private double turnScale() {
        return slowHeld() ? DriveConstants.SLOW_MODE_TURN_SCALE : 1.0;
    }

    public boolean aimLocked() {
        double error = limelight.getAimErrorDeg(chassis.getPose());
        return !Double.isNaN(error) && Math.abs(error) <= DriveConstants.AIM_FEED_TOLERANCE_DEG;
    }

    private void rumbleFeedback() {
        boolean ready = shooter.atTargetVelocity();
        boolean locked = aimHeld() && aimLocked();
        if (DriveConstants.RUMBLE_FEEDBACK) {
            if (ready && !wasShooterReady) rawGamepad2.rumble(200);
            if (locked && !wasAimLocked) rawGamepad1.rumble(150);
        }
        wasShooterReady = ready;
        wasAimLocked = locked;
    }

    public void stop() {
        ROBOT_POSE = chassis.getPose();
        limelight.stop();
    }

    public void setPose(Pose pose) {
        chassis.setPose(pose);
    }

    public Command followPath(Path path) {
        return ChassisCommands.createFollowPathCommand(chassis, path);
    }

    public Command followPath(Path path, boolean holdEnd) {
        return ChassisCommands.createFollowPathCommand(chassis, path, holdEnd);
    }

    public Command enableShooter() {
        return ShooterCommands.enableShooter(shooter);
    }

    public Command aim() {
        return ChassisCommands.createAimCommand(chassis, limelight);
    }

    public Command aimAndShoot(long shootTime) {
        return Commands.sequence(
                ShooterCommands.enableShooter(shooter),
                Commands.race(
                        Commands.waitUntil(limelight::hasTarget),
                        Commands.waitMillis(AutoConstants.TARGET_WAIT_TIME)),
                aim(),
                Commands.race(
                        Commands.waitUntil(shooter::atTargetVelocity),
                        Commands.waitMillis(AutoConstants.MAX_SPIN_UP_TIME)),
                ShooterCommands.shoot(shooter),
                Commands.waitMillis(shootTime),
                ShooterCommands.disableShooter(shooter));
    }

    public Command shootFor(long shootTime) {
        return aimAndShoot(shootTime);
    }

    public Chassis getChassis() {
        return chassis;
    }

    public Limelight getLimelight() {
        return limelight;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void initLoop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        chassis.periodic();
        limelight.periodic();
        PanelsDrawing.update();
        telemetry.update();
    }

    public void periodic() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        run();
        if (teleop) rumbleFeedback();

        PanelsDrawing.update();
        telemetry.addLine();
        if (teleop) {
            telemetry.addData("Aim Locked", aimLocked());
            telemetry.addData("Slow Mode", slowHeld());
        }
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();
    }
}
