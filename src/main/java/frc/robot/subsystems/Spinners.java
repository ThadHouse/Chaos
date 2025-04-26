package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.motorcontrol.SparkMini;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinners extends SubsystemBase {
    private final SparkFlex leftSpinner;
    private final SparkMini rightSpinner;

    public Spinners() {
        leftSpinner = new SparkFlex(4, 4, MotorType.kBrushed);

        SparkFlexConfig config = new SparkFlexConfig();

        leftSpinner.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightSpinner = new SparkMini(5);
    }

    public void setLeft(double speed) {
        leftSpinner.set(speed);
    }

    public void setRight(double speed) {
        rightSpinner.set(speed);
    }
}
