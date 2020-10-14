package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.bodyFrameToWorldFrame;

public class SLAMManager
{
   private double xDebug = Double.NaN;
   private double yDebug = Double.NaN;

   private static final double dt = 0.01;
   private static final double initialX = 1.5;
   private static final double initialY = 1.5;
   private static final double correctionMultiplier = 0.33;

   private double velocity;
   private double heading;
   private final Point2D xyPosition = new Point2D(initialX, initialY);
   private final Vector2D xyVelocity = new Vector2D();
   private final Vector2D xyHeading = new Vector2D();
   private final List<Point2D> wallPoints = new ArrayList<>();

   private ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame;

   public SLAMManager()
   {
      wallPoints.add(new Point2D(0.0, 0.0));
      wallPoints.add(new Point2D(0.0, 10.0));
      wallPoints.add(new Point2D(10.0, 10.0));
      wallPoints.add(new Point2D(10.0, 0.0));
   }

   public void update()
   {
      xyHeading.setX(-Math.sin(heading));
      xyHeading.setY(Math.cos(heading));

      xyVelocity.set(xyHeading);
      xyVelocity.scale(velocity);

      xyPosition.addX(xyVelocity.getX() * dt);
      xyPosition.addY(xyVelocity.getY() * dt);

      doLocalization();
   }

   private void doLocalization()
   {
      int numberOfCorrections = 0;
      Vector2D totalCorrectionVector = new Vector2D();

      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> sensorReading = vectorsAndDistancesToWallInBodyFrame.get(i);

         Point2D wallPointInBody = new Point2D(sensorReading.getLeft());
         wallPointInBody.scale(1.0 / wallPointInBody.distanceFromOrigin());
         wallPointInBody.scale(sensorReading.getRight());

         Point2D wallPointInWorldEstimatedFrame = new Point2D();
         bodyFrameToWorldFrame(wallPointInBody, wallPointInWorldEstimatedFrame, heading, xyPosition);

         Vector2D sensorDirectionInBody = new Vector2D(sensorReading.getLeft());
         Vector2D sensorDirectionInWorld = new Vector2D();
         bodyFrameToWorldFrame(sensorDirectionInBody, sensorDirectionInWorld, heading);

         for (int j = 0; j < wallPoints.size(); j++)
         {
            Point2D expectedIntersectionPoint = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(xyPosition,
                                                                                                             sensorDirectionInWorld,
                                                                                                             wallPoints.get(j),
                                                                                                             wallPoints.get((j + 1) % wallPoints.size()));
            if (expectedIntersectionPoint != null)
            {
               numberOfCorrections++;

               Vector2D correctionVector = new Vector2D(expectedIntersectionPoint);
               correctionVector.sub(wallPointInWorldEstimatedFrame);
               totalCorrectionVector.add(correctionVector);

               break;
            }
         }
      }

      if (numberOfCorrections > 0)
      {
         totalCorrectionVector.scale(correctionMultiplier / numberOfCorrections);
         xyPosition.add(totalCorrectionVector);
      }

      if (!Double.isNaN(xDebug))
      {
         double xError = Math.abs(xDebug - xyPosition.getX());
         double yError = Math.abs(yDebug - xyPosition.getY());
         System.out.println("X Error = " + xError + "\t\t Y Error = " + yError);
      }
   }

   public void setVectorsAndDistancesToWallInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      this.vectorsAndDistancesToWallInBodyFrame = vectorsAndDistancesToWallInBodyFrame;
   }

   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      this.xDebug = x;
      this.yDebug = y;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   public void setHeading(double heading)
   {
      this.heading = heading;
   }

   public double getVelocity()
   {
      return velocity;
   }

   public double getHeading()
   {
      return heading;
   }

   public Point2D getXYPosition()
   {
      return xyPosition;
   }

   public Vector2D getXYVelocity()
   {
      return xyVelocity;
   }

   public Vector2D getXYHeading()
   {
      return xyHeading;
   }
}
