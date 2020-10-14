package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.*;
import java.util.function.Function;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import javax.swing.*;
import java.util.ArrayList;

public class DuncanRobot05Behavior implements Robot05Behavior, Robot06Behavior
{
   private boolean paused = false;
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private ArrayList<Pair<Vector2D, Double>> sensors;
   private ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood;
   private ArrayList<Pair<Point2D, Vector2D>> locationOfAllPredators;
   private Pair<Point2D, Integer> closestFlag;

   private AlphaFilter velocityFilter = new AlphaFilter(30.0);
   private AlphaFilter headingFilter = new AlphaFilter(15.0);

   SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Robot"),
                                                                 new NullGraphics3DAdapter(),
                                                                 new SimulationConstructionSetParameters());
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
//   private final YoRegistry yoRegistry = scs.getRootRegistry();
   private final YoDouble headingAngle = new YoDouble("HeadingAngle", yoRegistry);
   private final YoVector2D headingVector = new YoVector2D("HeadingVector", yoRegistry);
   private final YoPoint2D me = new YoPoint2D("Me", yoRegistry);
   private final YoDouble noisyVelocity = new YoDouble("NoisyVelocity", yoRegistry);
   private final YoDouble velocity = new YoDouble("Velocity", yoRegistry);
   private final YoDouble noisyHeading = new YoDouble("NoisyHeading", yoRegistry);
   private final List<AlphaFilter> sensorFilters = new ArrayList<>();
   private final List<YoDouble> wallErrors = new ArrayList<>();
   {
      for (int i = 0; i < 9; i++)
      {
         YoDouble wallError = new YoDouble("WallError" + i, yoRegistry);
         wallErrors.add(wallError);
         sensorFilters.add(new AlphaFilter(15.0));
      }
   }
   private final List<Point2D> lastFoodPositions = new ArrayList<>();
   private final List<YoPoint2D> yoPredators = new ArrayList<>();
   private final List<AlphaFilter> predatorPositionFiltersX = new ArrayList<>();
   private final List<AlphaFilter> predatorPositionFiltersY = new ArrayList<>();
   private final List<YoPoint2D> filteredYoPredators = new ArrayList<>();
   {
      for (int i = 0; i < 3; i++)
      {
         yoPredators.add(new YoPoint2D("Predator" + i, yoRegistry));
         filteredYoPredators.add(new YoPoint2D("FilteredPredator" + i, yoRegistry));
         predatorPositionFiltersX.add(new AlphaFilter(10.0));
         predatorPositionFiltersY.add(new AlphaFilter(10.0));
      }
   }
   private final YoVector2D slamCorrection = new YoVector2D("SlamCorrection", yoRegistry);

   public DuncanRobot05Behavior()
   {
      scs.addYoRegistry(yoRegistry);
      scs.setDT(1.0, 1);
      scs.setCameraFix(0.4, 0.0, 0.0);
      scs.setupGraph(noisyVelocity.getName(), velocity.getName());
      String[] sensorErrors = new String[wallErrors.size()];
      for (int i = 0; i < wallErrors.size(); i++)
      {
         sensorErrors[i] = wallErrors.get(i).getName();
      }
      scs.setupGraph(sensorErrors);
      String[] predatorGraphsX = new String[6];
      String[] predatorGraphsY = new String[6];
      for (int i = 0; i < 3; i++)
      {
         predatorGraphsX[i * 2] = yoPredators.get(i).getYoX().getName();
         predatorGraphsX[i * 2 + 1] = filteredYoPredators.get(i).getYoX().getName();
         predatorGraphsY[i * 2] = yoPredators.get(i).getYoY().getName();
         predatorGraphsY[i * 2 + 1] = filteredYoPredators.get(i).getYoY().getName();
      }
      scs.setupGraph(predatorGraphsX);
      scs.setupGraph(predatorGraphsY);
      scs.setupGraph(noisyHeading.getName(), headingAngle.getName());
      scs.setupGraph(headingVector.getYoX().getName(), headingVector.getYoY().getName());
      scs.setupGraph(me.getYoX().getName(), me.getYoY().getName());
      scs.setupGraph(slamCorrection.getYoX().getName(), slamCorrection.getYoY().getName());
      scs.hideViewport();
      scs.changeBufferSize(4096);
      JToggleButton pauseButton = new JToggleButton("Pause");
      pauseButton.addActionListener(e -> paused = pauseButton.isSelected());
      scs.addButton(pauseButton);

      scs.startOnAThread();
      while (!scs.hasSimulationThreadStarted())
         ThreadTools.sleep(200);
   }

   @Override
   public void senseVelocity(double rawVelocity)
   {
      noisyVelocity.set(rawVelocity);
      velocity.set(velocityFilter.filter(rawVelocity));
   }

   @Override
   public void senseHeading(double rawHeading)
   {
      noisyHeading.set(rawHeading);
      headingAngle.set(headingFilter.filter(rawHeading));
   }
   @Override
   public void senseWallRangeInBodyFrame(ArrayList<Pair<Vector2D, Double>> vectorsAndDistancesToWallInBodyFrame)
   {
      ArrayList<Pair<Vector2D, Double>> filteredSensors = new ArrayList<>();
      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> sensor = vectorsAndDistancesToWallInBodyFrame.get(i);
         filteredSensors.add(Pair.of(sensor.getLeft(), sensorFilters.get(i).filter(sensor.getRight())));
      }

      this.sensors = filteredSensors;
   }

   @Override
   public void senseHitWall()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> locationOfAllFood)
   {
//      ArrayList<Pair<Point2D, Vector2D>> filteredFood = new ArrayList<>();
//      for (int i = 0; i < locationOfAllFood.size(); i++)
//      {
//         Pair<Point2D, Vector2D> food = locationOfAllFood.get(i);
//         Point2D filteredLocation = new Point2D();
//         double alpha = 10.0;
//         filteredLocation.setX(alphaFilter(food.getLeft().getX(), lastFoodPositions.get(i).getX(), alpha));
//         filteredLocation.setY(alphaFilter(food.getLeft().getY(), lastFoodPositions.get(i).getY(), alpha));
//         lastFoodPositions.get(i).set(filteredLocation);
//         filteredFood.add(Pair.of(filteredLocation, food.getRight()));
//      }
      this.locationOfAllFood = locationOfAllFood;
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> rawPredators)
   {
      ArrayList<Pair<Point2D, Vector2D>> filteredPredators = new ArrayList();
      for (int i = 0; i < rawPredators.size(); i++)
      {
         Pair<Point2D, Vector2D> predator = rawPredators.get(i);
         yoPredators.get(i).set(predator.getLeft());
         Point2D filteredLocation = new Point2D();
         filteredLocation.setX(predatorPositionFiltersX.get(i).filter(predator.getLeft().getX()));
         filteredLocation.setY(predatorPositionFiltersY.get(i).filter(predator.getLeft().getY()));
         filteredYoPredators.get(i).set(filteredLocation);
         filteredPredators.add(Pair.of(filteredLocation, predator.getRight()));
      }

      this.locationOfAllPredators = filteredPredators;
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      if (carrying != flagId)
      {
         carrying = -1;
         LogTools.info("Dropped flag: {} Going for: {}", flagId, currentFlagId);
      }
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      carrying = id;
      LogTools.info("Picked up: {} Going for: {}", carrying, currentFlagId);
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      if (carrying == flagId)
      {
         carrying = -1;
         LogTools.info("Goal! Flag: {}", currentFlagId);
         if (currentFlagId < 5)
         {
            currentFlagId++;
         }
         else
         {
            currentFlagId = 1;
         }
         LogTools.info("Next flag: {}", currentFlagId);
      }
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> vectorToInBodyFrameAndIdOfClosestFlag)
   {
      this.closestFlag = vectorToInBodyFrameAndIdOfClosestFlag;
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      this.mousePressedX = mousePressedX;
      this.mousePressedY = mousePressedY;
   }

   int currentFlagId = 1;
   int carrying = -1;

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      headingVector.set(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(headingAngle.getValue());
      transform.transform(headingVector);

      // 0 heading is y+ (up)
      double dt = 0.01;
      me.add(dt * velocity.getValue() * headingVector.getX(),
             dt * velocity.getValue() * headingVector.getY());

      double fieldGraduation = 1.5;
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);
      Vector2D attractionVector = new Vector2D();

      Vector2D meToMouse = fieldVector(me, mouse, distance -> 10.0 * Math.pow(distance, 1.5));
      Point2D center = new Point2D(5.0, 5.0);
      Vector2D meToCenter = fieldVector(me, center, distance -> 2.0 * Math.pow(distance, 1.5));

      Line2D left = new Line2D(0.0, 0.0, 0.0, 1.0);
      Line2D right = new Line2D(10.0, 0.0, 0.0, 1.0);
      Line2D bottom = new Line2D(0.0, 0.0, 1.0, 0.0);
      Line2D top = new Line2D(0.0, 10.0, 1.0, 0.0);
      List<Line2D> walls = Arrays.asList(left, right, bottom, top);

      slamCorrection.setToZero();
      ArrayList<Point2D> estimatedHits = new ArrayList<>();
      ArrayList<Point2D> actualHits = new ArrayList<>();
      for (int i = 0; i < sensors.size(); i++)
      {
         Pair<Vector2D, Double> sensor = sensors.get(i);

         Vector2D scanRayWorld = bodyToWorld(sensor.getLeft());
//         Vector2D scanRayWorld = sensor.getLeft();
         Point2D estimatedIntersection = new Point2D();
         double closest = 100.0;
         for (Line2D wall : walls)
         {
            Point2D potentialEstimatedIntersection = new Point2D();
            wall.intersectionWith(new Line2D(me, scanRayWorld), potentialEstimatedIntersection);

            Vector2D toIntersection = new Vector2D();
            toIntersection.sub(potentialEstimatedIntersection, me);
            if (toIntersection.dot(scanRayWorld) > 0.0 && potentialEstimatedIntersection.distance(me) < closest)
            {
               estimatedIntersection = potentialEstimatedIntersection;
               closest = estimatedIntersection.distance(me);
            }
         }

         estimatedHits.add(estimatedIntersection);

         Point2D actualHit = new Point2D(me);
         Vector2D hitMovement = new Vector2D(scanRayWorld);
         hitMovement.scale(sensor.getRight());
         actualHit.add(hitMovement);
         actualHits.add(actualHit);

         wallErrors.get(i).set(estimatedHits.get(i).distance(actualHits.get(i)));

         slamCorrection.add(estimatedHits.get(i).getX() - actualHits.get(i).getX(),
                            estimatedHits.get(i).getY() - actualHits.get(i).getY());
      }

      slamCorrection.scale(1.0 / sensors.size());
//      me.add(slamCorrection);

      Vector2D boundaryRepulsion = new Vector2D();
      double boundaryStrength = 2.0;
      Point2D closestLeft = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, left.getPoint(), left.getDirection());
      Point2D closestRight = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, right.getPoint(), right.getDirection());
      Point2D closestBottom = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, bottom.getPoint(), bottom.getDirection());
      Point2D closestTop = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, top.getPoint(), top.getDirection());
      boundaryRepulsion.add(fieldVector(closestLeft, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));
      boundaryRepulsion.add(fieldVector(closestRight, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));
      boundaryRepulsion.add(fieldVector(closestBottom, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));
      boundaryRepulsion.add(fieldVector(closestTop, me, distance -> boundaryStrength / Math.pow(distance, fieldGraduation)));

      Vector2D predatorRepulsion = new Vector2D();
      for (Pair<Point2D, Vector2D> predator : locationOfAllPredators)
      {
         predatorRepulsion.add(fieldVector(bodyToWorld(predator.getLeft()), me, distance -> 6.0 / Math.pow(distance, fieldGraduation)));
      }
//      predatorRepulsion.scale(1.0 / locationOfAllPredators.size());

      Vector2D foodAttraction = new Vector2D();
      for (Triple<Integer, Point2D, Vector2D> food : locationOfAllFood)
      {
         foodAttraction.add(fieldVector(me, bodyToWorld(food.getMiddle()), distance -> 0.5 / Math.pow(distance, 1.5)));
      }

      if (closestFlag != null)
      {
         Vector2D flagField = new Vector2D();
         if (closestFlag != null && closestFlag.getRight() == currentFlagId)
         {
            if (carrying != currentFlagId) // toward flag to pick up
            {
               flagField.add(fieldVector(me, bodyToWorld(closestFlag.getLeft()), distance -> 6.0 / Math.pow(distance, fieldGraduation)));
            }
         }
         else
         {
            flagField.add(fieldVector(bodyToWorld(closestFlag.getLeft()), me, distance -> 3.0 / Math.pow(distance, 2.0)));
         }

         if (carrying == currentFlagId) // set to goal
         {
            flagField.add(fieldVector(me, new Point2D(9.0, 9.0), distance -> 15.0 / Math.pow(distance, 0.5)));
         }
         attractionVector.add(flagField);
      }

//      attractionVector.add(meToMouse);
      attractionVector.add(meToCenter);
      attractionVector.add(boundaryRepulsion);
      attractionVector.add(predatorRepulsion);
      attractionVector.add(foodAttraction);

      double desiredSpeed = attractionVector.length();

      double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                    headingVector.getY(),
                                                                                    attractionVector.getX(),
                                                                                    attractionVector.getY());

      double acceleration = (1.0 * (desiredSpeed - velocity.getValue()));

      double angularVelocity = (velocity.getValue() - lastVelocity) / dt;
      double turnRate = (5.0 * angleToAttraction) + (-0.5 * angularVelocity);
      lastVelocity = velocity.getValue();

      if (Double.isNaN(acceleration)) acceleration = 0.0;
      if (Double.isNaN(turnRate)) turnRate = 0.0;

      if (!paused)
         scs.tickAndUpdate();

      return new double[] {acceleration, turnRate};
   }

   double lastVelocity = 0.0;

   private Point2D bodyToWorld(Point2DReadOnly pointInBody)
   {
      Point2D pointInWorld = new Point2D(pointInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-headingAngle.getValue(), 0.0, 0.0);
      pointInWorld.applyInverseTransform(transform);
      pointInWorld.add(me);
      return pointInWorld;
   }

   private Vector2D bodyToWorld(Vector2DReadOnly vectorInBody)
   {
      Vector2D vectorInWorld = new Vector2D(vectorInBody);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().setYawPitchRoll(-headingAngle.getValue(), 0.0, 0.0);
      vectorInWorld.applyInverseTransform(transform);
      return vectorInWorld;
   }

   private Vector2D fieldVector(Tuple2DReadOnly from, Tuple2DReadOnly to, Function<Double, Double> magnitude)
   {
      Vector2D vector = new Vector2D(to);
      vector.sub(from);
      double distance = vector.length();
      vector.normalize();
      vector.scale(magnitude.apply(distance));
      return vector;
   }

   static class AlphaFilter
   {
      private final double alpha;
      private double last = Double.NaN;

      public AlphaFilter(double alpha)
      {
         this.alpha = alpha;
      }

      public double filter(double value)
      {
         double filtered;
         if (Double.isNaN(last)) // feed forward to initial value
         {
            last = value;
            filtered = value;
         }
         else
         {
            filtered = last + (value - last) / alpha;
            last = filtered;
         }
         return filtered;
      }
   }

   @Override
   public boolean getDropFlag()
   {
      return ((me.getX() > 8.0) && (me.getY() > 8.0));
   }
}
