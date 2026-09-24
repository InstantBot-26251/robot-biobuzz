package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower createAutonomous(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h,localizerConfig),
                new Mecanum(h, autonomousConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower createTeleop(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h,localizerConfig),
                new Mecanum(h, teleOpConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig autonomousConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(false);
    });

    public static MecanumConfig teleOpConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });



    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(0.987169399555226);
        c.angularScalar.set(0.9797871068054063);
        c.offset.set(new Pose(-0.0, -0.0, 90));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    // TODO: actually tune foresight
    // https://pedropathing.com/docs/pathing/tuning/foresight
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15002441617562762);
                Controller secondaryTranslationalForward = Controller.proportional(0.05543001305066442);
                Controller primaryTranslationalLateral = Controller.proportional(0.1760022017992246);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06502811069936751);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01963401237423452));
                c.brake.set(Controller.proportionalFeedforward(0.016688910518099342));

                c.headingFeedback.set(Controller.proportional(2.0572890009769838));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.22042024994518794, -0.04450045988429643));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08369903522095223, 0.0531251500824898));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011016318835190934, 0.0018709596984412223));

                c.maxAchievableForwardVelocity.set(54.54269770864734);
                c.maxAchievableStrafeVelocity.set(45.88244426376259);
                c.naturalForwardDeceleration.set(49.09135408008957);
                c.naturalStrafeDeceleration.set(68.20366381951034);
            }
    );
}