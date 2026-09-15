package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("topRight");
        c.frontRightName.set("topLeft");
        c.backLeftName.set("bottomRight");
        c.backRightName.set("bottomLeft");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // need to re-tune pinpoint localization since it was plugged into I2C 0 instead of 1
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("imu");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.8049670241949127);
        c.yPodOffset.set(-2.3847314128725547);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3939124731096849);
                Controller secondaryTranslationalForward = Controller.proportional(0.1455401332788955);
                Controller primaryTranslationalLateral = Controller.proportional(0.823556999277943);
                Controller secondaryTranslationalLateral = Controller.proportional(0.3042823053848918);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014588341625357387));
                c.brake.set(Controller.proportionalFeedforward(0.012400090381553779));

                c.headingFeedback.set(Controller.proportional(8.78531402144574));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.058225880339940424, 0.0036370394378758826));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10418419037760093, 0.04803262131753152));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012584341316735671, 0.002197707241969356));

                c.maxAchievableForwardVelocity.set(69.04840609859575);
                c.maxAchievableStrafeVelocity.set(52.87766999748821);
                c.naturalForwardDeceleration.set(33.38645287462979);
                c.naturalStrafeDeceleration.set(61.45128114469674);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}