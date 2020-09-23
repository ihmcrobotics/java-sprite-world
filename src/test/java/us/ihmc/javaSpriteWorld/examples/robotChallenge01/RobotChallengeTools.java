package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotChallengeTools
{
   public static Point2D computePositionInRobotBodyFrame(Point2D robotLocationInWorld, Point2D pointOfInterestInWorld, double robotHeading)
   {
      Vector2D vectorFromRobotInWorld = computeVectorFromRobotInWorld(robotLocationInWorld, pointOfInterestInWorld);
      Point2D vectorInBody = computePositionInBodyFrame(vectorFromRobotInWorld, robotHeading);
      return vectorInBody;
   }

   public static Vector2D computeVectorFromRobotInWorld(Point2D robotLocationInWorld, Point2D pointOfInterestInWorld)
   {
      Vector2D vectorFromRobotInWorld = new Vector2D(pointOfInterestInWorld);
      vectorFromRobotInWorld.sub(robotLocationInWorld);

      return vectorFromRobotInWorld;
   }

   public static Point2D computePositionInBodyFrame(Vector2D vectorInWorld, double robotHeading)
   {
      Point2D vectorInBody = new Point2D(vectorInWorld);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(robotHeading, 0.0, 0.0);
      vectorInBody.applyInverseTransform(transform);

      return vectorInBody;
   }
}
