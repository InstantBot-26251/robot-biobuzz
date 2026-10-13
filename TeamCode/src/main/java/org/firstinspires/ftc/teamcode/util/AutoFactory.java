package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.RobotContainer;

public abstract class AutoFactory extends OpMode {
    protected final PoseFactory poseFactory = PoseFactory.degrees();
    protected Pose startPose;
    protected RobotContainer robot;

    protected abstract Alliance getAlliance();
    protected abstract Pose setStartPose();
    protected abstract Command createAutoSequence();

    @Override
    public void init() {
        Alliance.setCurrent(getAlliance());
        startPose = setStartPose();

        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.autonomousInit();

        robot.setPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", Alliance.getCurrent());
        telemetry.update();
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(createAutoSequence());
    }

    @Override
    public void loop() {

        robot.periodic();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
