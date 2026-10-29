package frc.robot.subsystems.pathplanner;

import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public interface PathIO {
  default PathPlannerPath getPath(PathingConstants.PathName path)
      throws FileVersionException, IOException, ParseException {
    return PathPlannerPath.fromPathFile(path.filename);
  }

  default Command createPathCommand(
      PathingConstants.PathName pathName,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedSupplier,
      Consumer<ChassisSpeeds> output)
      throws FileVersionException, IOException, ParseException {
    return AutoBuilder.followPath(getPath(pathName));
  }
}
