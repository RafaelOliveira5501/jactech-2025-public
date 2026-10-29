package frc.robot.subsystems.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public class PathIOReal implements PathIO {
  // Add hardware-specific path loading if needed
  // (e.g., different paths for competition vs practice bot)

  @Override
  public Command createPathCommand(
      PathingConstants.PathName pathName,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedSupplier,
      Consumer<ChassisSpeeds> output)
      throws FileVersionException, IOException, ParseException {
    return AutoBuilder.followPath(getPath(pathName));
  }
}
