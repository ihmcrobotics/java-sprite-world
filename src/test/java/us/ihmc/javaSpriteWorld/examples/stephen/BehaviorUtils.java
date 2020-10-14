package us.ihmc.javaSpriteWorld.examples.stephen;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.*;

public class BehaviorUtils
{
   public static int clamp(int x, int min, int max)
   {
      return Math.min(Math.max(x, min), max);
   }

   public static double filter(double alpha, double value, double previous)
   {
      return alpha * value + (1.0 - alpha) * previous;
   }

   public static double headingFromVector(Tuple2DReadOnly vector)
   {
      return headingFromVector(vector.getX(), vector.getY());
   }

   public static double headingFromVector(double x, double y)
   {
      return Math.atan2(-x, y);
   }

   public static void bodyFrameToWorldFrame(Point2DReadOnly bodyFramePoint, Point2DBasics worldFramePointToSet, double heading, Point2D xyPosition)
   {
      double relativeWorldVectorX = bodyFramePoint.getX() * Math.cos(heading) - bodyFramePoint.getY() * Math.sin(heading);
      double relativeWorldVectorY = bodyFramePoint.getX() * Math.sin(heading) + bodyFramePoint.getY() * Math.cos(heading);

      worldFramePointToSet.setX(relativeWorldVectorX + xyPosition.getX());
      worldFramePointToSet.setY(relativeWorldVectorY + xyPosition.getY());
   }

   public static void worldFrameToBodyFrame(Point2DReadOnly worldFramePoint, Point2DBasics bodyFramePointToSet, double heading, Point2D xyPosition)
   {
      double dx = worldFramePoint.getX() - xyPosition.getX();
      double dy = worldFramePoint.getY() - xyPosition.getY();

      bodyFramePointToSet.setX(dx * Math.cos(heading) + dy * Math.sin(heading));
      bodyFramePointToSet.setY(-dx * Math.sin(heading) + dy * Math.cos(heading));
   }

   public static void bodyFrameToWorldFrame(Vector2DReadOnly bodyFrameVector, Vector2DBasics worldFrameVectorToSet, double heading)
   {
      worldFrameVectorToSet.setX(bodyFrameVector.getX() * Math.cos(heading) - bodyFrameVector.getY() * Math.sin(heading));
      worldFrameVectorToSet.setY(bodyFrameVector.getX() * Math.sin(heading) + bodyFrameVector.getY() * Math.cos(heading));
   }

   public static void worldFrameToBodyFrame(Vector2DReadOnly worldFrameVector, Vector2DBasics bodyFrameVectorToSet, double heading)
   {
      bodyFrameVectorToSet.setX(worldFrameVector.getX() * Math.cos(heading) + worldFrameVector.getY() * Math.sin(heading));
      bodyFrameVectorToSet.setY(-worldFrameVector.getX() * Math.sin(heading) + worldFrameVector.getY() * Math.cos(heading));
   }

}
