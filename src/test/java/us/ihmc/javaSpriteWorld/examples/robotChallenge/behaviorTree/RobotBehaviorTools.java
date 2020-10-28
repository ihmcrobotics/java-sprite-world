package us.ihmc.javaSpriteWorld.examples.robotChallenge.behaviorTree;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.function.Function;

public class RobotBehaviorTools
{
   public static Point2D bodyToWorld(RobotBehaviorSensors sensors, Point2DReadOnly pointInBody)
   {
      Point2D pointInWorld = new Point2D(pointInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-sensors.getHeading(), 0.0, 0.0);
      pointInWorld.applyInverseTransform(transform);
      pointInWorld.add(sensors.getGlobalPosition());
      return pointInWorld;
   }

   public static Vector2D bodyToWorld(RobotBehaviorSensors sensors, Vector2DReadOnly vectorInBody)
   {
      Vector2D vectorInWorld = new Vector2D(vectorInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-sensors.getHeading(), 0.0, 0.0);
      vectorInWorld.applyInverseTransform(transform);
      return vectorInWorld;
   }

   public static Vector2D fieldVector(Tuple2DReadOnly from, Tuple2DReadOnly to, Function<Double, Double> magnitude)
   {
      Vector2D vector = new Vector2D(to);
      vector.sub(from);
      double distance = vector.length();
      vector.normalize();
      vector.scale(magnitude.apply(distance));
      return vector;
   }
}
