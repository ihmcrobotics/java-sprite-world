package us.ihmc.javaSpriteWorld.examples.stephen;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.bodyFrameToWorldFrame;
import static us.ihmc.javaSpriteWorld.examples.stephen.BehaviorUtils.filter;

public class SLAMManager
{
   private double xDebug = Double.NaN;
   private double yDebug = Double.NaN;
   private double headingDebug = Double.NaN;
   private final int debugPrintFrequency = 10;
   private int debugPrintCounter = 0;

   public static final double dt = 0.01;
   private static final double initialX = 1.5;
   private static final double initialY = 1.5;
   private static final double correctionMultiplier = 0.3;
   private static final double alphaVelocity = 0.25;
   private static final double alphaHeading = 0.25;

   private double sensedVelocity;
   private double sensedHeading;

   private double filteredVelocity;
   private double filteredHeading;

   private final Point2D xyPosition = new Point2D(initialX, initialY);
   private final Vector2D xyVelocity = new Vector2D();
   private final Vector2D xyHeading = new Vector2D();
   private final List<Point2D> wallPoints = new ArrayList<>();

   private ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame;
   private final double[] actionFromLastTick = new double[2];

   public SLAMManager()
   {
      wallPoints.add(new Point2D(0.0, 0.0));
      wallPoints.add(new Point2D(0.0, 10.0));
      wallPoints.add(new Point2D(10.0, 10.0));
      wallPoints.add(new Point2D(10.0, 0.0));
   }

   public void setActionFromLastTick(double[] actionFromLastTick)
   {
      for (int i = 0; i < 2; i++)
      {
         this.actionFromLastTick[i] = actionFromLastTick[i];
      }
   }

   public void update()
   {
      filteredHeading = filter(alphaHeading, sensedHeading, filteredHeading);
      filteredVelocity = filter(alphaVelocity, sensedVelocity, filteredVelocity);

      xyHeading.setX(-Math.sin(filteredHeading));
      xyHeading.setY(Math.cos(filteredHeading));

      xyVelocity.set(xyHeading);
      xyVelocity.scale(filteredVelocity);

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
         bodyFrameToWorldFrame(wallPointInBody, wallPointInWorldEstimatedFrame, filteredHeading, xyPosition);

         Vector2D sensorDirectionInBody = new Vector2D(sensorReading.getLeft());
         Vector2D sensorDirectionInWorld = new Vector2D();
         bodyFrameToWorldFrame(sensorDirectionInBody, sensorDirectionInWorld, filteredHeading);

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

      if (!Double.isNaN(xDebug) && debugPrintCounter++ > debugPrintFrequency)
      {
         debugPrintCounter = 0;
         double xError = Math.abs(xDebug - xyPosition.getX());
         double yError = Math.abs(yDebug - xyPosition.getY());
         double errorMagnitude = EuclidCoreTools.norm(xError, yError);
         double angularError = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(headingDebug, filteredHeading));
         System.out.println("Position error = " + errorMagnitude + "\t Heading error = " + angularError);
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

   public void senseNoiseFreeHeadingForTestingOnly(double headingDebug)
   {
      this.headingDebug = headingDebug;
   }

   public void senseVelocity(double sensedVelocity)
   {
      this.sensedVelocity = sensedVelocity;
   }

   public void senseHeading(double sensedHeading)
   {
      this.sensedHeading = sensedHeading;
   }

   public double getFilteredVelocity()
   {
      return filteredVelocity;
   }

   public double getHeading()
   {
      return filteredHeading;
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
