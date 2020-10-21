package us.ihmc.javaSpriteWorld.examples.robotChallenge05;

import java.util.*;
import java.util.function.Function;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.*;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.javaSpriteWorld.examples.robotChallenge06.Robot06Behavior;
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import javax.swing.*;
import java.util.ArrayList;

public class DuncanRobot05Behavior implements Robot05Behavior, Robot06Behavior
{
   private boolean paused = false;
   private boolean initializedValues = false;
   private double mousePressedX = 5.0, mousePressedY = 5.0;
   private ArrayList<Pair<Vector2D, Double>> sensors;
   private ArrayList<Pair<Vector2D, Double>> noisySensors;
   private ArrayList<Triple<Integer, Point2D, Vector2D>> foods;
   private ArrayList<Pair<Point2DBasics, Vector2D>> predators;
   private Pair<Point2D, Integer> closestFlag;

   private AlphaFilter velocityFilter = new AlphaFilter(25.0);
   private AlphaFilter headingFilter = new AlphaFilter(15.0);

   SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Robot"),
                                                                 new NullGraphics3DAdapter(),
                                                                 new SimulationConstructionSetParameters());
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
//   private final YoRegistry yoRegistry = scs.getRootRegistry();
   private final YoDouble headingAngle = new YoDouble("HeadingAngle", yoRegistry);
   private final YoDouble groundTruthHeading = new YoDouble("GroundTruthHeading", yoRegistry);
   private final YoVector2D headingVector = new YoVector2D("HeadingVector", yoRegistry);
   private final YoPoint2D groundTruthPosition = new YoPoint2D("GroundTruthPosition", yoRegistry);
   private final YoPoint2D me = new YoPoint2D("Me", yoRegistry);
   private final YoDouble noisyVelocity = new YoDouble("NoisyVelocity", yoRegistry);
   private final YoDouble velocity = new YoDouble("Velocity", yoRegistry);
   private final YoDouble noisyHeading = new YoDouble("NoisyHeading", yoRegistry);
   private final YoDouble acceleration = new YoDouble("Acceleration", yoRegistry);
   private final YoDouble turnRate = new YoDouble("TurnRate", yoRegistry);
   private final List<AlphaFilter> sensorFilters = new ArrayList<>();
   private final List<YoPoint2D> estimatedHits = new ArrayList<>();
   private final List<YoPoint2D> actualHits = new ArrayList<>();
   private final List<YoPoint2D> noisyHits = new ArrayList<>();
   private final List<YoVector2D> hitErrors = new ArrayList<>();
   public static final int NUMBER_OF_SENSORS = 9;
   {
      for (int i = 0; i < NUMBER_OF_SENSORS; i++)
      {
         estimatedHits.add(new YoPoint2D("EstimatedHit" + i, yoRegistry));
         actualHits.add(new YoPoint2D("ActualHit" + i, yoRegistry));
         noisyHits.add(new YoPoint2D("NoisyHit" + i, yoRegistry));
         hitErrors.add(new YoVector2D("HitError" + i, yoRegistry));
         sensorFilters.add(new AlphaFilter(3.0));
      }
   }
   private static final int NUMBER_OF_PREDATORS = 3;
   private final List<YoPoint2D> yoPredators = new ArrayList<>();
   private final List<AlphaFilteredTuple2D> predatorPositionFilters = new ArrayList<>();
   private final List<YoPoint2D> filteredYoPredators = new ArrayList<>();
   {
      for (int i = 0; i < NUMBER_OF_PREDATORS; i++)
      {
         yoPredators.add(new YoPoint2D("Predator" + i, yoRegistry));
         YoPoint2D filtered = new YoPoint2D("FilteredPredator" + i, yoRegistry);
         filteredYoPredators.add(filtered);
         predatorPositionFilters.add(new AlphaFilteredTuple2D(filtered, 10.0));
      }
   }
   public static final int NUMBER_OF_FOOD = 10;
   private final List<YoPoint2D> yoNoisyFood = new ArrayList<>();
   private final List<YoPoint2D> yoFilteredFood = new ArrayList<>();
   private final Map<Integer, Pair<AlphaFilter, AlphaFilter>> foodFilters = new HashMap<>();
   {
      for (int i = 0; i < NUMBER_OF_FOOD; i++)
      {
         yoNoisyFood.add(new YoPoint2D("NoisyFood" + i, yoRegistry));
         yoFilteredFood.add(new YoPoint2D("FilteredFood" + i, yoRegistry));
      }
   }
   private final YoVector2D slamCorrection = new YoVector2D("SlamCorrection", yoRegistry);
   private final AlphaFilteredTuple2D slamCorrectionFilter = new AlphaFilteredTuple2D(slamCorrection, 20.0);
   private final YoVector2D boundaryRepulsion = new YoVector2D("BoundaryRepulsion", yoRegistry);
   private final ClampAlphaFilteredTuple2D boundaryFilter = new ClampAlphaFilteredTuple2D(boundaryRepulsion, 20.0, 2.0);
   private final YoVector2D predatorRepulsion = new YoVector2D("PredatorRepulsion", yoRegistry);
   private final ClampAlphaFilteredTuple2D predatorFilter = new ClampAlphaFilteredTuple2D(predatorRepulsion, 20.0, 2.0);
   private final YoVector2D foodAttraction = new YoVector2D("FoodAttraction", yoRegistry);
   private final ClampAlphaFilteredTuple2D foodFilter = new ClampAlphaFilteredTuple2D(foodAttraction, 20.0, 2.0);
   private final YoVector2D flagField = new YoVector2D("FlagField", yoRegistry);
   private final ClampAlphaFilteredTuple2D flagFilter = new ClampAlphaFilteredTuple2D(flagField, 20.0, 2.0);
   private final YoVector2D attractionVector = new YoVector2D("Attraction", yoRegistry);
   private final AlphaFilteredTuple2D attractionFilter = new AlphaFilteredTuple2D(attractionVector, 20.0);
   private final YoDouble boundaryRepulsionMagnitude = new YoDouble("BoundaryRepulsionMagnitude", yoRegistry);
   private final YoDouble predatorRepulsionMagnitude = new YoDouble("PredatorRepulsionMagnitude", yoRegistry);
   private final YoDouble foodAttractionMagnitude = new YoDouble("FoodAttractionMagnitude", yoRegistry);
   private final YoDouble flagFieldMagnitude = new YoDouble("FlagFieldMagnitude", yoRegistry);
   private final YoDouble attractionMagnitude = new YoDouble("AttractionMagnitude", yoRegistry);
   private final YoInteger yoClosestFlag = new YoInteger("ClosestFlag", yoRegistry);
   private final YoInteger carriedFlag = new YoInteger("CarriedFlag", yoRegistry);
   private final YoInteger goalFlag = new YoInteger("GoalFlag", yoRegistry);
   private final YoInteger goal = new YoInteger("Goal", yoRegistry);
   private final YoDouble distanceToGoal = new YoDouble("DistanceToGoal", yoRegistry);
   private final YoDouble health = new YoDouble("Health", yoRegistry);
   private final YoDouble score = new YoDouble("Score", yoRegistry);
   private final YoDouble time = new YoDouble("Time", yoRegistry);

   public DuncanRobot05Behavior()
   {
      scs.addYoRegistry(yoRegistry);
      scs.setDT(1.0, 1);
      scs.setCameraFix(0.4, 0.0, 0.0);
      GraphArrayWindow predatorGraphs = scs.createNewGraphWindow("Predators");
      for (int i = 0; i < NUMBER_OF_PREDATORS; i++)
      {
         predatorGraphs.setupGraph(new String[] {yoPredators.get(i).getYoX().getName(), filteredYoPredators.get(i).getYoX().getName()});
         predatorGraphs.setupGraph(new String[] {yoPredators.get(i).getYoY().getName(), filteredYoPredators.get(i).getYoY().getName()});
      }
      predatorGraphs.getGraphArrayPanel().addColumn();
      GraphArrayWindow foodGraphs = scs.createNewGraphWindow("Food");
      for (int i = 0; i < NUMBER_OF_FOOD; i++)
      {
         foodGraphs.setupGraph(new String[] {yoNoisyFood.get(i).getYoX().getName(), yoFilteredFood.get(i).getYoX().getName()});
         foodGraphs.setupGraph(new String[] {yoNoisyFood.get(i).getYoY().getName(), yoFilteredFood.get(i).getYoY().getName()});
      }
      foodGraphs.getGraphArrayPanel().addColumn();
      GraphArrayWindow sensorGraphs = scs.createNewGraphWindow("Sensors");
      String[] hitErrorsX = new String[NUMBER_OF_SENSORS];
      String[] hitErrorsY = new String[NUMBER_OF_SENSORS];
      for (int i = 0; i < NUMBER_OF_SENSORS; i++)
      {
         hitErrorsX[i] = hitErrors.get(i).getYoX().getName();
         hitErrorsY[i] = hitErrors.get(i).getYoY().getName();
         sensorGraphs.setupGraph(new String[] {noisyHits.get(i).getYoX().getName(),
                                 estimatedHits.get(i).getYoX().getName(),
                                 actualHits.get(i).getYoX().getName()});
         sensorGraphs.setupGraph(new String[] {noisyHits.get(i).getYoY().getName(),
                                 estimatedHits.get(i).getYoY().getName(),
                                 actualHits.get(i).getYoY().getName()});
      }
      sensorGraphs.setupGraph(hitErrorsX);
      sensorGraphs.setupGraph(hitErrorsY);
      sensorGraphs.getGraphArrayPanel().addColumn();
      scs.setupGraph(noisyVelocity.getName(), velocity.getName());
      scs.setupGraph(noisyHeading.getName(), headingAngle.getName(), groundTruthHeading.getName());
      scs.setupGraph(headingVector.getYoX().getName());
      scs.setupGraph(headingVector.getYoY().getName());
      scs.setupGraph(slamCorrection.getYoX().getName());
      scs.setupGraph(slamCorrection.getYoY().getName());
//      scs.setupGraph(boundaryRepulsion.getYoX().getName());
//      scs.setupGraph(boundaryRepulsion.getYoY().getName());
//      scs.setupGraph(predatorRepulsion.getYoX().getName());
//      scs.setupGraph(predatorRepulsion.getYoY().getName());
//      scs.setupGraph(foodAttraction.getYoX().getName());
//      scs.setupGraph(foodAttraction.getYoY().getName());
//      scs.setupGraph(flagField.getYoX().getName());
//      scs.setupGraph(flagField.getYoY().getName());
      scs.setupGraph(attractionVector.getYoX().getName());
      scs.setupGraph(attractionVector.getYoY().getName());
      scs.setupGraph(boundaryRepulsionMagnitude.getName(),
                     predatorRepulsionMagnitude.getName(),
                     foodAttractionMagnitude.getName(),
                     flagFieldMagnitude.getName(),
                     attractionMagnitude.getName());
      scs.setupGraph(me.getYoX().getName(), me.getYoY().getName(), groundTruthPosition.getYoX().getName(), groundTruthPosition.getYoY().getName());
      scs.setupGraph(acceleration.getName());
      scs.setupGraph(turnRate.getName());
      scs.setupGraph(new String[] {yoClosestFlag.getName(), goal.getName(), goalFlag.getName(), carriedFlag.getName()});
      scs.setupGraph(distanceToGoal.getName());
      scs.setupGraph(health.getName());
      scs.setupGraph(score.getName());
      scs.setupGraph(time.getName());
      scs.getGUI().getGraphArrayPanel().addColumn();
      scs.skipLoadingDefaultConfiguration();
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
      noisySensors = vectorsAndDistancesToWallInBodyFrame;
      sensors = new ArrayList<>();

      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
      {
         Pair<Vector2D, Double> sensor = vectorsAndDistancesToWallInBodyFrame.get(i);
         sensors.add(Pair.of(sensor.getLeft(), sensorFilters.get(i).filter(sensor.getRight())));
      }
   }

   @Override
   public void senseHitWall()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void senseFoodInBodyFrame(ArrayList<Triple<Integer, Point2D, Vector2D>> rawFoods)
   {
      foods = new ArrayList<>();
      for (int i = 0; i < rawFoods.size(); i++)
      {
         Triple<Integer, Point2D, Vector2D> rawFood = rawFoods.get(i);
         foodFilters.computeIfAbsent(rawFood.getLeft(), index -> Pair.of(new AlphaFilter(10.0), new AlphaFilter(10.0)));
         Point2D filteredPosition = new Point2D();
         filteredPosition.setX(foodFilters.get(rawFood.getLeft()).getLeft().filter(rawFood.getMiddle().getX()));
         filteredPosition.setY(foodFilters.get(rawFood.getLeft()).getRight().filter(rawFood.getMiddle().getY()));
         yoNoisyFood.get(rawFood.getLeft() - 1).set(rawFood.getMiddle());
         yoFilteredFood.get(rawFood.getLeft() - 1).set(filteredPosition);
         foods.add(Triple.of(rawFood.getLeft(), filteredPosition, rawFood.getRight()));
      }
   }

   @Override
   public void sensePredatorsInBodyFrame(ArrayList<Pair<Point2D, Vector2D>> rawPredators)
   {
      predators = new ArrayList<>();
      for (int i = 0; i < rawPredators.size(); i++)
      {
         Pair<Point2D, Vector2D> predator = rawPredators.get(i);
         yoPredators.get(i).set(predator.getLeft());
         filteredYoPredators.get(i).set(predator.getLeft());
         predatorPositionFilters.get(i).filter();
         predators.add(Pair.of(filteredYoPredators.get(i), predator.getRight()));
      }
   }

   @Override
   public void senseDroppedFlag(int flagId)
   {
      if (carriedFlag.getValue() == flagId)
      {
         carriedFlag.set(-1);
         LogTools.info("Dropped flag: {} Going for: {}", flagId, goalFlag.getValue());
      }
   }

   @Override
   public void sensePickedUpFlag(int id)
   {
      if (carriedFlag.getValue() != id)
      {
         carriedFlag.set(id);
         LogTools.info("Picked up: {} Going for: {}", carriedFlag.getValue(), goalFlag.getValue());
      }
   }

   @Override
   public void senseDeliveredFlag(int flagId)
   {
      if (carriedFlag.getValue() == flagId)
      {
         carriedFlag.set(-1);
         LogTools.info("Goal! Flag: {}", goalFlag.getValue());
         if (goalFlag.getValue() < 5)
         {
            goalFlag.add(1);
         }
         else
         {
            goalFlag.set(1);
         }
         LogTools.info("Next flag: {}", goalFlag.getValue());
      }
   }

   @Override
   public void senseClosestFlagInBodyFrame(Pair<Point2D, Integer> vectorToInBodyFrameAndIdOfClosestFlag)
   {
      closestFlag = vectorToInBodyFrameAndIdOfClosestFlag;
      if (closestFlag != null && closestFlag.getRight() == carriedFlag.getValue()) // means we missed a drop somehow
      {
         carriedFlag.set(-1);
      }
      yoClosestFlag.set(closestFlag.getRight());
   }

   @Override
   public void senseMousePressed(double mousePressedX, double mousePressedY)
   {
      this.mousePressedX = mousePressedX;
      this.mousePressedY = mousePressedY;
   }

   private void initializeValues()
   {
      initializedValues = true;
      me.set(1.5, 1.5);
      goalFlag.set(1);
      carriedFlag.set(-1);
   }

   @Override
   public double[] getAccelerationAndTurnRate()
   {
      headingVector.set(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(headingAngle.getValue());
      transform.transform(headingVector);

      if (!initializedValues) initializeValues();

      // 0 heading is y+ (up)
      double dt = 0.01;
      me.add(dt * velocity.getValue() * headingVector.getX(),
             dt * velocity.getValue() * headingVector.getY());

      double fieldGraduation = 1.5;
      Vector2D mouse = new Vector2D(mousePressedX, mousePressedY);

      Vector2D meToMouse = fieldVector(me, mouse, distance -> 10.0 * Math.pow(distance, 1.5));
      Point2D center = new Point2D(5.0, 5.0);
      Vector2D meToCenter = fieldVector(me, center, distance -> 7.0 * Math.pow(distance, 1.5));

      Line2D left = new Line2D(0.0, 0.0, 0.0, 1.0);
      Line2D right = new Line2D(10.0, 0.0, 0.0, 1.0);
      Line2D bottom = new Line2D(0.0, 0.0, 1.0, 0.0);
      Line2D top = new Line2D(0.0, 10.0, 1.0, 0.0);
      List<Line2D> walls = Arrays.asList(left, right, bottom, top);

      slamCorrection.setToZero();
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

         estimatedHits.get(i).set(estimatedIntersection);

         Point2D actualHit = new Point2D(me);
         Vector2D hitMovement = new Vector2D(scanRayWorld);
         hitMovement.scale(sensor.getRight());
         actualHit.add(hitMovement);
         actualHits.get(i).set(actualHit);

         Point2D noisyHit = new Point2D(me);
         Vector2D noisyHitMovement = new Vector2D(scanRayWorld);
         noisyHitMovement.scale(noisySensors.get(i).getRight());
         noisyHit.add(noisyHitMovement);
         noisyHits.get(i).set(noisyHit);

         hitErrors.get(i).set(estimatedHits.get(i).getX() - actualHits.get(i).getX(),
                              estimatedHits.get(i).getY() - actualHits.get(i).getY());

         // should add up the "wrench" of each instead
         slamCorrection.add(hitErrors.get(i));
      }
      slamCorrection.scale(0.01 * 1.0 / NUMBER_OF_SENSORS);
      slamCorrectionFilter.filter();
      me.add(slamCorrection);

      boundaryRepulsion.setToZero();
      double boundaryStrength = 2.0;
      double boundaryGraduation = 1.5;
      Point2D closestLeft = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, left.getPoint(), left.getDirection());
      Point2D closestRight = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, right.getPoint(), right.getDirection());
      Point2D closestBottom = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, bottom.getPoint(), bottom.getDirection());
      Point2D closestTop = EuclidGeometryTools.orthogonalProjectionOnLine2D(me, top.getPoint(), top.getDirection());
      boundaryRepulsion.add(fieldVector(closestLeft, me, distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
      boundaryRepulsion.add(fieldVector(closestRight, me, distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
      boundaryRepulsion.add(fieldVector(closestBottom, me, distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
      boundaryRepulsion.add(fieldVector(closestTop, me, distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));

      predatorRepulsion.setToZero();
      for (Pair<Point2DBasics, Vector2D> predator : predators)
      {
         predatorRepulsion.add(fieldVector(bodyToWorld(predator.getLeft()), me, distance -> 2.0 / Math.pow(distance, 1.8)));
      }
//      predatorRepulsion.scale(1.0 / locationOfAllPredators.size());

      foodAttraction.setToZero();
      for (Triple<Integer, Point2D, Vector2D> food : foods)
      {
         foodAttraction.add(fieldVector(me, bodyToWorld(food.getMiddle()), distance -> 0.5 / Math.pow(distance, 1.5)));
      }

      attractionVector.setToZero();

      flagField.setToZero();
      if (closestFlag != null)
      {
         if (closestFlag.getRight() == goalFlag.getValue())
         {
            if (carriedFlag.getValue() != goalFlag.getValue()) // attract toward next flag to pick up
            {
               flagField.add(fieldVector(me, bodyToWorld(closestFlag.getLeft()), distance ->
               {
                  distanceToGoal.set(distance);
                  return 12.0 / Math.pow(distance, fieldGraduation);
               }));
            }
         }
         else // repulse from out-of-order flag
         {
            flagField.add(fieldVector(bodyToWorld(closestFlag.getLeft()), me, distance -> 0.5 / Math.pow(distance, 1.5)));
         }
      }
      if (carriedFlag.getValue() == goalFlag.getValue()) // attract to goal; no flags nearby
      {
         flagField.add(fieldVector(me, new Point2D(9.0, 9.0), distance ->
         {
            distanceToGoal.set(distance);
            return 15.0 / Math.pow(distance, 0.5);
         }));
      }
      else if (closestFlag == null) // looking for a flag i.e. wander
      {
         attractionVector.add(meToCenter); // TODO: Change from center
      }
      if (carriedFlag.getValue() == goalFlag.getValue())
      {
         goal.set(10);
      }
      else
      {
         goal.set(goalFlag.getValue());
      }
      flagFilter.filter();
      attractionVector.add(flagField);

      boundaryFilter.filter();
      predatorFilter.filter();
      foodFilter.filter();
//      attractionVector.add(meToMouse);
//      attractionVector.add(meToCenter);
      attractionVector.add(boundaryRepulsion);
      attractionVector.add(predatorRepulsion);
      attractionVector.add(foodAttraction);

      attractionFilter.filter();

      boundaryRepulsionMagnitude.set(boundaryRepulsion.length());
      predatorRepulsionMagnitude.set(predatorRepulsion.length());
      foodAttractionMagnitude.set(foodAttraction.length());
      flagFieldMagnitude.set(flagField.length());
      attractionMagnitude.set(attractionVector.length());

      double desiredSpeed = attractionVector.length();

      double angleToAttraction = EuclidGeometryTools.angleFromFirstToSecondVector2D(headingVector.getX(),
                                                                                    headingVector.getY(),
                                                                                    attractionVector.getX(),
                                                                                    attractionVector.getY());

      acceleration.set(1.0 * (desiredSpeed - velocity.getValue()));

      double angularVelocity = (velocity.getValue() - lastVelocity) / dt;
      turnRate.set((5.0 * angleToAttraction) + (-0.5 * angularVelocity));
      lastVelocity = velocity.getValue();

      double accelerationToReturn = acceleration.getValue();
      double turnRateToReturn = turnRate.getValue();
      if (Double.isNaN(acceleration.getValue())) accelerationToReturn = 0.0;
      if (Double.isNaN(turnRate.getValue())) turnRateToReturn = 0.0;

      if (!paused)
         scs.tickAndUpdate();

      return new double[] {accelerationToReturn, turnRateToReturn};
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

   static class ClampAlphaFilteredTuple2D extends AlphaFilteredTuple2D
   {
      private double clamp;

      public ClampAlphaFilteredTuple2D(Tuple2DBasics tuple, double alpha, double clamp)
      {
         super(tuple, alpha);
         this.clamp = clamp;
      }

      @Override
      public void filter()
      {
         tuple.set(xFilter.filter(MathTools.clamp(tuple.getX(), clamp)), yFilter.filter(MathTools.clamp(tuple.getY(), clamp)));
      }
   }

   static class AlphaFilteredTuple2D
   {
      protected final Tuple2DBasics tuple;
      protected final AlphaFilter xFilter;
      protected final AlphaFilter yFilter;

      public AlphaFilteredTuple2D(Tuple2DBasics tuple, double alpha)
      {
         this.tuple = tuple;
         this.xFilter = new AlphaFilter(alpha);
         this.yFilter = new AlphaFilter(alpha);
      }

      public void filter()
      {
         tuple.set(xFilter.filter(tuple.getX()), yFilter.filter(tuple.getY()));
      }
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
   public void senseKeyPressed(String keyPressed)
   {

   }

   @Override
   public boolean getDropFlag()
   {
      return ((me.getX() > 8.0) && (me.getY() > 8.0));
   }

   @Override
   public void senseScoreHealthTime(double score, double health, double elapsedTime)
   {
      this.score.set(score);
      this.health.set(health);
      this.time.set(elapsedTime);
   }

   @Override
   public void reset()
   {

   }

   @Override
   public void senseGlobalPositionForTestingOnly(double x, double y)
   {
      groundTruthPosition.set(x, y);
   }

   @Override
   public void senseNoiseFreeHeadingForTestingOnly(double heading)
   {
      groundTruthHeading.set(heading);
   }
}
