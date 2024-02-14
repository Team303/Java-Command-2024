package frc.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.IMUProtocol.StreamResponse;;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;


public class NavXModule {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double yawVelocityRadPerSec = 0.0;
    private final AHRS navX = new AHRS();


    public NavXModule(){
        navX.reset();
        navX.getActualUpdateRate();
        StreamResponse lol = new StreamResponse();
        lol.update_rate_hz=150;
    }

  public default void updateInputs(GyroIOInputs inputs) {}
}