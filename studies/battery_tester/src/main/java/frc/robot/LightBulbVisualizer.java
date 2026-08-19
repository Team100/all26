package frc.robot;

import java.util.function.DoubleSupplier;

import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

public class LightBulbVisualizer {
    private final DoubleSupplier m_temperature;
    private final MechanismLigament2d m_filament;

    /**
     * @param temperature kelvin
     */
    public LightBulbVisualizer(DoubleSupplier temperature) {
        m_temperature = temperature;
        Mechanism2d m2d = new Mechanism2d(100, 100);
        m_filament = new MechanismLigament2d(
                "filament", 100, 0, 200, new Color8Bit(Color.BLACK));
        m2d.getRoot("root", 0, 50).append(m_filament);
        SmartDashboard.putData("lightbulb", m2d);
    }

    public void periodic() {
        m_filament.setColor(Zubetto.color(m_temperature.getAsDouble()));
    }

}