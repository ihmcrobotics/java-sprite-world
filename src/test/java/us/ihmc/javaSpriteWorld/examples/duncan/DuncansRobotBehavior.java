package us.ihmc.javaSpriteWorld.examples.duncan;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.yoVariables.euclid.YoPoint2D;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import javax.swing.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class DuncansRobotBehavior extends FunctionalRobotBehaviorAdapter
{
   private final Environment environment;
   private boolean paused = false;
   private boolean initializedValues = false;
   private Point2D mousePressedPosition = new Point2D(1.0, 1.0);
   private ArrayList<Pair<Vector2D, Double>> sensors;
   private ArrayList<Pair<Vector2D, Double>> noisySensors;
   private ArrayList<Triple<Integer, Point2D, Vector2D>> foods;
   private ArrayList<Pair<Point2DBasics, Vector2D>> predators;
   private Pair<Point2D, Integer> closestFlag;

   private AlphaFilter velocityFilter = new AlphaFilter(25.0);
   private AlphaFilter headingFilter = new AlphaFilter(15.0);
   double lastVelocity = 0.0; // PD controller

   private final boolean runSCS;
   private int numberOfFlags;
   private int numberOfPredators;
   private int numberOfFood;
   private SimulationConstructionSet scs;
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
   private final List<YoPoint2D> yoPredators = new ArrayList<>();
   private final List<AlphaFilteredTuple2D> predatorPositionFilters = new ArrayList<>();
   private final List<YoPoint2D> filteredYoPredators = new ArrayList<>();
   private final List<YoPoint2D> yoNoisyFood = new ArrayList<>();
   private final List<YoPoint2D> yoFilteredFood = new ArrayList<>();
   private final Map<Integer, Pair<AlphaFilter, AlphaFilter>> foodFilters = new HashMap<>();
   private final YoVector2D slamCorrection = new YoVector2D("SlamCorrection", yoRegistry);
   private final AlphaFilteredTuple2D slamCorrectionFilter = new AlphaFilteredTuple2D(slamCorrection, 20.0);
   private final YoVector2D boundaryRepulsion = new YoVector2D("BoundaryRepulsion", yoRegistry);
   private final ClampAlphaFilteredTuple2D boundaryFilter = new ClampAlphaFilteredTuple2D(boundaryRepulsion, 20.0, 8.0);
   private final YoVector2D predatorRepulsion = new YoVector2D("PredatorRepulsion", yoRegistry);
   private final ClampAlphaFilteredTuple2D predatorFilter = new ClampAlphaFilteredTuple2D(predatorRepulsion, 20.0, 4.0);
   private final YoVector2D foodAttraction = new YoVector2D("FoodAttraction", yoRegistry);
   private final ClampAlphaFilteredTuple2D foodFilter = new ClampAlphaFilteredTuple2D(foodAttraction, 20.0, 2.0);
   private final YoVector2D flagField = new YoVector2D("FlagField", yoRegistry);
   private final ClampAlphaFilteredTuple2D flagFilter = new ClampAlphaFilteredTuple2D(flagField, 20.0, 2.0);
   private final YoVector2D wanderField = new YoVector2D("WanderField", yoRegistry);
   private final YoVector2D attractionVector = new YoVector2D("Attraction", yoRegistry);
   private final AlphaFilteredTuple2D attractionFilter = new AlphaFilteredTuple2D(attractionVector, 20.0);
   private final YoDouble boundaryRepulsionMagnitude = new YoDouble("BoundaryRepulsionMagnitude", yoRegistry);
   private final YoDouble predatorRepulsionMagnitude = new YoDouble("PredatorRepulsionMagnitude", yoRegistry);
   private final YoDouble foodAttractionMagnitude = new YoDouble("FoodAttractionMagnitude", yoRegistry);
   private final YoDouble flagFieldMagnitude = new YoDouble("FlagFieldMagnitude", yoRegistry);
   private final YoDouble wanderFieldMagnitude = new YoDouble("WanderFieldMagnitude", yoRegistry);
   private final YoDouble attractionMagnitude = new YoDouble("AttractionMagnitude", yoRegistry);
   private final YoInteger yoClosestFlag = new YoInteger("ClosestFlag", yoRegistry);
   private final YoInteger carriedFlag = new YoInteger("CarriedFlag", yoRegistry);
   private final YoInteger goalFlag = new YoInteger("GoalFlag", yoRegistry);
   private final YoInteger goal = new YoInteger("Goal", yoRegistry);
   private final YoInteger wanderMode = new YoInteger("WanderMode", yoRegistry);
   private final YoBoolean wandering = new YoBoolean("Wandering", yoRegistry);
   private final YoDouble distanceToGoal = new YoDouble("DistanceToGoal", yoRegistry);
   private final YoDouble health = new YoDouble("Health", yoRegistry);
   private final YoDouble score = new YoDouble("Score", yoRegistry);
   private final YoDouble time = new YoDouble("Time", yoRegistry);

   public DuncansRobotBehavior(int challengeNumber,
                               boolean runSCS,
                               int numberOfFlags,
                               int numberOfPredators,
                               int numberOfFood)
   {
      environment = new Environment(challengeNumber);
      this.runSCS = runSCS;
      this.numberOfFlags = numberOfFlags;
      this.numberOfPredators = numberOfPredators;
      this.numberOfFood = numberOfFood;
      setupYoVariables();
      if (runSCS)
         setupSCS();
      setSenseVelocity(rawVelocity ->
                       {
                          noisyVelocity.set(rawVelocity);
                          velocity.set(velocityFilter.filter(rawVelocity));
                       });
      setSenseHeading(rawHeading ->
                      {
                         noisyHeading.set(rawHeading);
                         headingAngle.set(headingFilter.filter(rawHeading));
                      });
      setSenseWallRangeInBodyFrame(vectorsAndDistancesToWallInBodyFrame ->
                                   {
                                      noisySensors = vectorsAndDistancesToWallInBodyFrame;
                                      sensors = new ArrayList<>();

                                      for (int i = 0; i < vectorsAndDistancesToWallInBodyFrame.size(); i++)
                                      {
                                         Pair<Vector2D, Double> sensor = vectorsAndDistancesToWallInBodyFrame.get(i);
                                         sensors.add(Pair.of(sensor.getLeft(), sensorFilters.get(i).filter(sensor.getRight())));
                                      }
                                   });
      setSenseFoodInBodyFrame(rawFoods ->
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
                              });
      setSensePredatorsInBodyFrame(rawPredators ->
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
                                   });
      setSenseCarryingFlag(carriedFlag::set);
      setSenseDeliveredFlag(id ->
                            {
                               if (goalFlag.getValue() < numberOfFlags)
                               {
                                  goalFlag.add(1);
                               }
                               else
                               {
                                  goalFlag.set(1);
                               }
                            });
      setSenseClosestFlagInBodyFrame(vectorToInBodyFrameAndIdOfClosestFlag ->
                                     {
                                        closestFlag = vectorToInBodyFrameAndIdOfClosestFlag;
                                        if (closestFlag != null && closestFlag.getRight() == carriedFlag.getValue()) // means we missed a drop somehow
                                        {
                                           carriedFlag.set(-1);
                                        }
                                        yoClosestFlag.set(closestFlag.getRight());
                                     });
      setSenseMousePressed(mousePressedPosition ->
                           {
                              this.mousePressedPosition = mousePressedPosition;
                           });
      setGetAccelerationAndTurnRate(this::doControl);
   }

   private void setupYoVariables()
   {
      for (int i = 0; i < NUMBER_OF_SENSORS; i++)
      {
         estimatedHits.add(new YoPoint2D("EstimatedHit" + i, yoRegistry));
         actualHits.add(new YoPoint2D("ActualHit" + i, yoRegistry));
         noisyHits.add(new YoPoint2D("NoisyHit" + i, yoRegistry));
         hitErrors.add(new YoVector2D("HitError" + i, yoRegistry));
         sensorFilters.add(new AlphaFilter(3.0));
      }
      for (int i = 0; i < numberOfPredators; i++)
      {
         yoPredators.add(new YoPoint2D("Predator" + i, yoRegistry));
         YoPoint2D filtered = new YoPoint2D("FilteredPredator" + i, yoRegistry);
         filteredYoPredators.add(filtered);
         predatorPositionFilters.add(new AlphaFilteredTuple2D(filtered, 10.0));
      }
      for (int i = 0; i < numberOfFood; i++)
      {
         yoNoisyFood.add(new YoPoint2D("NoisyFood" + i, yoRegistry));
         yoFilteredFood.add(new YoPoint2D("FilteredFood" + i, yoRegistry));
      }
   }

   private void initializeValues()
   {
      initializedValues = true;
      me.set(1.5, 1.5);
      goalFlag.set(1);
      carriedFlag.set(-1);
   }

   private Pair<Double, Double> doControl()
   {
      headingVector.set(0.0, 1.0);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getRotation().appendYawRotation(headingAngle.getValue());
      transform.transform(headingVector);

      if (!initializedValues)
         initializeValues();

      // 0 heading is y+ (up)
      double dt = 0.01;
      me.add(dt * velocity.getValue() * headingVector.getX(), dt * velocity.getValue() * headingVector.getY());

      double fieldGraduation = 1.5;

      Vector2D meToMouse = fieldVector(me, mousePressedPosition, distance -> 10.0 * Math.pow(distance, 1.5));
      Point2D center = new Point2D(5.0, 5.0);
      Vector2D meToCenter = fieldVector(me, center, distance -> 7.0 * Math.pow(distance, 1.5));

      List<LineSegment2D> walls = environment.getWalls();

      slamCorrection.setToZero();
      for (int i = 0; i < sensors.size(); i++)
      {
         Pair<Vector2D, Double> sensor = sensors.get(i);

         Vector2D scanRayWorld = bodyToWorld(sensor.getLeft());
         //         Vector2D scanRayWorld = sensor.getLeft();
         Point2D estimatedIntersection = new Point2D();
         double closest = 100.0;
         for (LineSegment2D wall : walls)
         {
            Point2D potentialEstimatedIntersection = new Point2D();
            boolean intersects = wall.intersectionWith(new Line2D(me, scanRayWorld), potentialEstimatedIntersection);

            if (intersects)
            {
               Vector2D toIntersection = new Vector2D();
               toIntersection.sub(potentialEstimatedIntersection, me);
               if (toIntersection.dot(scanRayWorld) > 0.0 && potentialEstimatedIntersection.distance(me) < closest)
               {
                  estimatedIntersection = potentialEstimatedIntersection;
                  closest = estimatedIntersection.distance(me);
               }
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

         hitErrors.get(i).set(estimatedHits.get(i).getX() - actualHits.get(i).getX(), estimatedHits.get(i).getY() - actualHits.get(i).getY());

         // should add up the "wrench" of each instead
         slamCorrection.add(hitErrors.get(i));
      }
      slamCorrection.scale(0.01 * 1.0 / NUMBER_OF_SENSORS);
      slamCorrectionFilter.filter();
      me.add(slamCorrection);

      boundaryRepulsion.setToZero();
      double boundaryStrength = 3.0;
      double boundaryGraduation = 2.5;
      for (LineSegment2D wall : walls)
      {
         Point2D closest = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(me, wall.getFirstEndpoint(), wall.getSecondEndpoint());
         boundaryRepulsion.add(fieldVector(closest, me, distance -> boundaryStrength / Math.pow(distance, boundaryGraduation)));
      }

      predatorRepulsion.setToZero();
      for (Pair<Point2DBasics, Vector2D> predator : predators)
      {
         predatorRepulsion.add(fieldVector(bodyToWorld(predator.getLeft()), me, distance -> 7.0 / Math.pow(distance, 3.0)));
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
            flagField.add(fieldVector(bodyToWorld(closestFlag.getLeft()), me, distance -> 2.0 / Math.pow(distance, 1.5)));
         }
      }
      wandering.set(false);
      wanderField.setToZero();
      if (carriedFlag.getValue() == goalFlag.getValue()) // attract to goal; no flags nearby
      {
         flagField.add(fieldVector(me, new Point2D(environment.getMapSizeX() * 0.9, environment.getMapSizeY() * 0.9), distance ->
         {
            distanceToGoal.set(distance);
            return 15.0 / Math.pow(distance, 0.5);
         }));
      }
      else if (closestFlag != null && closestFlag.getRight() != goalFlag.getValue() && predatorRepulsion.length() < 2.0) // looking for a flag i.e. wander
      {
         wandering.set(true);
         int second = (int) Math.floor(time.getValue()) / 7;
         wanderMode.set(second % 5);
         Function<Double, Double> magnitude = distance -> 1.0 * Math.pow(distance, 1.0);
         switch (wanderMode.getValue())
         {
            case 0:
               wanderField.add(fieldVector(me, new Point2D(environment.getMapSizeX() * 0.5, environment.getMapSizeY() * 0.5), magnitude));
               break;
            case 1:
               wanderField.add(fieldVector(me, new Point2D(environment.getMapSizeX() * 0.9, environment.getMapSizeY() * 0.9), magnitude));
               break;
            case 2:
               wanderField.add(fieldVector(me, new Point2D(environment.getMapSizeX() * 0.9, environment.getMapSizeY() * 0.1), magnitude));
               break;
            case 3:
               wanderField.add(fieldVector(me, new Point2D(environment.getMapSizeX() * 1.0, environment.getMapSizeY() * 0.1), magnitude));
               break;
            default:
               wanderField.add(fieldVector(me, new Point2D(environment.getMapSizeX() * 0.1, environment.getMapSizeY() * 0.9), magnitude));
               break;
         }
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
      attractionVector.add(wanderField);
      attractionVector.add(boundaryRepulsion);
      attractionVector.add(predatorRepulsion);
      attractionVector.add(foodAttraction);

      attractionFilter.filter();

      boundaryRepulsionMagnitude.set(boundaryRepulsion.length());
      predatorRepulsionMagnitude.set(predatorRepulsion.length());
      foodAttractionMagnitude.set(foodAttraction.length());
      flagFieldMagnitude.set(flagField.length());
      wanderFieldMagnitude.set(wanderField.length());
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
      if (Double.isNaN(acceleration.getValue()))
         accelerationToReturn = 0.0;
      if (Double.isNaN(turnRate.getValue()))
         turnRateToReturn = 0.0;

      if (runSCS && !paused)
         scs.tickAndUpdate();

      return Pair.of(accelerationToReturn, turnRateToReturn);
   }

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

   private void setupSCS()
   {
      scs = new SimulationConstructionSet(new Robot("Robot"), new NullGraphics3DAdapter(), new SimulationConstructionSetParameters());
      scs.addYoRegistry(yoRegistry);
      scs.setDT(1.0, 1);
      scs.setCameraFix(0.4, 0.0, 0.0);
      GraphArrayWindow predatorGraphs = scs.createNewGraphWindow("Predators");
      for (int i = 0; i < numberOfPredators; i++)
      {
         predatorGraphs.setupGraph(new String[] {yoPredators.get(i).getYoX().getName(), filteredYoPredators.get(i).getYoX().getName()});
         predatorGraphs.setupGraph(new String[] {yoPredators.get(i).getYoY().getName(), filteredYoPredators.get(i).getYoY().getName()});
      }
      predatorGraphs.getGraphArrayPanel().addColumn();
      GraphArrayWindow foodGraphs = scs.createNewGraphWindow("Food");
      for (int i = 0; i < numberOfFood; i++)
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
         sensorGraphs.setupGraph(new String[] {noisyHits.get(i).getYoX().getName(), estimatedHits.get(i).getYoX().getName(), actualHits.get(i).getYoX().getName()});
         sensorGraphs.setupGraph(new String[] {noisyHits.get(i).getYoY().getName(), estimatedHits.get(i).getYoY().getName(), actualHits.get(i).getYoY().getName()});
      }
      sensorGraphs.setupGraph(hitErrorsX);
      sensorGraphs.setupGraph(hitErrorsY);
      sensorGraphs.getGraphArrayPanel().addColumn();
      scs.setupGraph(noisyVelocity.getName(), velocity.getName());
      scs.setupGraph(noisyHeading.getName(), headingAngle.getName(), groundTruthHeading.getName());
      //      scs.setupGraph(headingVector.getYoX().getName());
      //      scs.setupGraph(headingVector.getYoY().getName());
      //      scs.setupGraph(slamCorrection.getYoX().getName());
      //      scs.setupGraph(slamCorrection.getYoY().getName());
      //      scs.setupGraph(boundaryRepulsion.getYoX().getName());
      //      scs.setupGraph(boundaryRepulsion.getYoY().getName());
      //      scs.setupGraph(predatorRepulsion.getYoX().getName());
      //      scs.setupGraph(predatorRepulsion.getYoY().getName());
      //      scs.setupGraph(foodAttraction.getYoX().getName());
      //      scs.setupGraph(foodAttraction.getYoY().getName());
      //      scs.setupGraph(flagField.getYoX().getName());
      //      scs.setupGraph(flagField.getYoY().getName());
      //      scs.setupGraph(attractionVector.getYoX().getName());
      //      scs.setupGraph(attractionVector.getYoY().getName());
      scs.setupGraph(boundaryRepulsionMagnitude.getName(),
                     predatorRepulsionMagnitude.getName(),
                     foodAttractionMagnitude.getName(),
                     flagFieldMagnitude.getName(),
                     wanderFieldMagnitude.getName(),
                     attractionMagnitude.getName());
      scs.setupGraph(me.getYoX().getName(), me.getYoY().getName(), groundTruthPosition.getYoX().getName(), groundTruthPosition.getYoY().getName());
      scs.setupGraph(acceleration.getName());
      scs.setupGraph(turnRate.getName());
      scs.setupGraph(new String[] {yoClosestFlag.getName(), goal.getName(), goalFlag.getName(), carriedFlag.getName()});
      scs.setupGraph(distanceToGoal.getName());
      scs.setupGraph(new String[] {wanderMode.getName(), wandering.getName()});
      scs.setupGraph(health.getName());
      scs.setupGraph(score.getName());
      scs.setupGraph(time.getName());
      scs.getGUI().getGraphArrayPanel().addColumn();
      scs.skipLoadingDefaultConfiguration();
      scs.hideViewport();
      scs.changeBufferSize(4096);
      JToggleButton pauseButton = new JToggleButton("Pause");
      pauseButton.addActionListener(e ->
                                    {
                                       paused = pauseButton.isSelected();
                                       scs.setScrollGraphsEnabled(paused);
                                    });
      scs.addButton(pauseButton);

      scs.startOnAThread();
      while (!scs.hasSimulationThreadStarted())
         ThreadTools.sleep(200);
   }
}
