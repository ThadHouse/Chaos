package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.Bank;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;

import edu.wpi.first.math.MathUtil;

public class ServoHubMotorController {
    private final ServoHub hub;

    private final ServoChannel frontLeft;
    private final ServoChannel frontRight;

    private final ServoChannel rearLeft;
    private final ServoChannel rearRight;

    public ServoHubMotorController() {
        hub = new ServoHub(3);

        ServoHubConfig config = new ServoHubConfig();
        PulseRange range = new PulseRange(500, 1500, 2500);
        config.channel0.pulseRange(range);
        config.channel1.pulseRange(range);
        config.channel2.pulseRange(range);
        config.channel3.pulseRange(range);
        config.channel4.pulseRange(range);
        config.channel5.pulseRange(range);

        System.out.println(hub.configure(config, ResetMode.kResetSafeParameters));
        System.out.println(hub.setBankPulsePeriod(Bank.kBank0_2, 5050));
        System.out.println(hub.setBankPulsePeriod(Bank.kBank3_5, 5050));

        frontLeft = hub.getServoChannel(ChannelId.kChannelId0);
        frontRight = hub.getServoChannel(ChannelId.kChannelId4);
        rearLeft = hub.getServoChannel(ChannelId.kChannelId1);
        rearRight = hub.getServoChannel(ChannelId.kChannelId5);

        System.out.println(frontLeft.setEnabled(true));
        System.out.println(frontRight.setEnabled(true));
        System.out.println(rearLeft.setEnabled(true));
        System.out.println(rearRight.setEnabled(true));
    }

    public void setFrontLeft(double speed) {
        double clamped = MathUtil.clamp(speed, -1, 1);

        // -1, 1 to -1000, 1000
        clamped *= 1000;
        clamped += 1500;

        // -1000, 1000 to 500, 2500

        if (clamped > 1490 && clamped < 1510) {
            clamped = 1500;
        }

        frontLeft.setPulseWidth((int) clamped);
    }

    public void setFrontRight(double speed) {
        double clamped = MathUtil.clamp(speed, -1, 1);

        // Reverse
        clamped *= -1;

        // -1, 1 to -1000, 1000
        clamped *= 1000;
        clamped += 1500;

        // -1000, 1000 to 500, 2500

        if (clamped > 1490 && clamped < 1510) {
            clamped = 1500;
        }

        frontRight.setPulseWidth((int) clamped);
    }

    public void setRearLeft(double speed) {
        double clamped = MathUtil.clamp(speed, -1, 1);

        // -1, 1 to -1000, 1000
        clamped *= 1000;
        clamped += 1500;

        // -1000, 1000 to 500, 2500

        if (clamped > 1490 && clamped < 1510) {
            clamped = 1500;
        }

        rearLeft.setPulseWidth((int) clamped);
    }

    public void setRearRight(double speed) {
        double clamped = MathUtil.clamp(speed, -1, 1);

        // Reverse
        clamped *= -1;

        // -1, 1 to -1000, 1000
        clamped *= 1000;
        clamped += 1500;

        // -1000, 1000 to 500, 2500

        if (clamped > 1490 && clamped < 1510) {
            clamped = 1500;
        }

        rearRight.setPulseWidth((int) clamped);
    }
}
