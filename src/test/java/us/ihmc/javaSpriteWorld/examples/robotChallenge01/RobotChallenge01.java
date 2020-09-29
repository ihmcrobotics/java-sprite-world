package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteStage;
import us.ihmc.javaSpriteWorld.SpriteWorld;
import us.ihmc.javaSpriteWorld.SpriteWorldMouseListener;
import us.ihmc.javaSpriteWorld.SpriteWorldViewer;
import us.ihmc.javaSpriteWorld.SpriteWorldViewerUsingSwing;

public class RobotChallenge01
{
   private final double xMax, yMax;
   private final Random random;

   private final SpriteWorldViewerUsingSwing viewer;
   private final SpriteWorldMouseListener spriteWorldMouseListener;
   private final SpriteWorld spriteWorld;
   private final SpriteStage stage;
   private final RobotChallengeRobot robot;
   private final FoodList01 foodList;
   private final PredatorList01 predatorList;
   private final FlagList flagList;

   private final CollisionProcessor01 collisionProcessor;
   
   private RobotChallengeRules robotChallengeRules;
   private SpriteCollisionGroup collisionGroup;

   private boolean mousePressed;
   private double mousePressedX, mousePressedY;

   public RobotChallenge01(String name, RobotChallengeRobot robot, Random random, double xMax, double yMax)
   {
      this.robot = robot;
      this.random = random;
      this.xMax = xMax;
      this.yMax = yMax;

      foodList = new FoodList01(xMax, yMax);
      predatorList = new PredatorList01();
      flagList = new FlagList();

      viewer = new SpriteWorldViewerUsingSwing(name);

      viewer.setPreferredSizeInPixels(1000, 1000);
      viewer.setResizable(false);

      spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setBottomBorderY(0.0);
      spriteWorld.setRightBorderX(xMax);
      spriteWorld.setTopBorderY(yMax);

      stage = new SpriteStage(name + " Stage");
      //      StageBackdrop backgammonBoardBackdrop = SampleStageBackdrops.getBackgammonBoard();
      //      backgammonBoardBackdrop.setXReferencePercent(0.5);
      //      backgammonBoardBackdrop.setYReferencePercent(0.5);

      //      stage.addBackdrop(backgammonBoardBackdrop);
      //      spriteWorld.setStage(stage, true);

      spriteWorld.addSprite(robot.getSprite());

      collisionGroup = new SpriteCollisionGroup();
      collisionGroup.addSprite(robot.getSprite());

      viewer.setSpriteWorld(spriteWorld);
      viewer.createAndDisplayWindow();

      collisionProcessor = new CollisionProcessor01(robot,
                                                    foodList,
                                                    predatorList,
                                                    flagList,
                                                    random,
                                                    xMax,
                                                    yMax,
                                                    spriteWorld,
                                                    collisionGroup);
      collisionGroup.addSpriteCollisionListener(collisionProcessor);

      spriteWorldMouseListener = new SpriteWorldMouseListener()
      {
         @Override
         public void mouseReleasedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
         }

         @Override
         public void mousePressedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
            mousePressedX = xWorld;
            mousePressedY = yWorld;
            mousePressed = true;
         }

         @Override
         public void mouseMovedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
         }

         @Override
         public void mouseExitedWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
         }

         @Override
         public void mouseEnteredWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
         }

         @Override
         public void mouseDraggedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
         }

         @Override
         public void mouseClickedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld, int clickCount)
         {
         }
      };

      spriteWorld.attacheSpriteWorldMouseListener(spriteWorldMouseListener);
   }

   public void createSomeFood(int numberOfPieces)
   {
      for (int i = 0; i < numberOfPieces; i++)
      {
         createSomeFood();
      }
   }

   private void createSomeFood()
   {
      foodList.createSomeFood(random, xMax, yMax, spriteWorld, collisionGroup);
   }

   public void createSomePredators(int numberOfPredators, double maximumPredatorSpeed)
   {
      for (int i = 0; i < numberOfPredators; i++)
      {
         createAPredator(maximumPredatorSpeed);
      }
   }

   private void createAPredator(double maximumPredatorSpeed)
   {
      predatorList.createAPredator(random, xMax, yMax, maximumPredatorSpeed, spriteWorld, collisionGroup);
   }

   public void createSomeFlags(int numberOfFlags)
   {
      for (int id = 1; id <= numberOfFlags; id++)
      {
         createAFlag(id);
      }
   }

   private void createAFlag(int id)
   {
      double x = randomDoubleBetween(xMax * 0.1, xMax * 0.9);
      double y = randomDoubleBetween(yMax * 0.3, yMax * 0.9);
      flagList.createAFlag(id, x, y, spriteWorld, collisionGroup);
   }

   public void robotDroppedFlag(Flag flag)
   {
      Point2D position = robot.getPosition();
      if ((position.getX() > 0.8 * xMax) && (position.getY() > 0.8 * yMax))
      {
         System.out.println("Flag " + flag.getId() + "returned home!!");
         robotChallengeRules.deliveredFlag(flag.getId());
      }
      
      Vector2D headingVector = robot.getHeadingVector();
      headingVector.scale(-1.5);
      position.add(headingVector);
      flag.setLocation(position);
      flagList.addFlag(flag, spriteWorld, collisionGroup);
   }

   private double randomDoubleBetween(double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   public void runSimulation()
   {
      double dt = 0.01;

      while (true)
      {
         if (mousePressed)
         {
            robotChallengeRules.senseMousePressed(mousePressedX, mousePressedY);
            mousePressed = false;
         }

         robotChallengeRules.executeRules();

         foodList.doDynamicsAndUpdateSprites(dt);
         predatorList.doDynamicsAndUpdateSprites(robot.getPosition(), robot.getVelocityVector(), dt);
         robot.doDynamicsAndUpdateSprite(dt);
         collisionGroup.doCheckCollisions();

         
         if (isOutOfBounds(robot.getPosition()))
         {
            robot.hitWall();
            robotChallengeRules.hitWall();
         }
         
         
         try
         {
            Thread.sleep((long) (dt * 1000));
         }
         catch (InterruptedException e)
         {
         }

         viewer.update();
      }
   }

   private boolean isOutOfBounds(Point2D position)
   {
      double x = position.getX();
      double y = position.getY();
      
      if (x > xMax)
         return true;
      if (y > yMax)
         return true;

      if (x < 0.0)
         return true;
      if (y < 0.0)
         return true;

      return false;
   }
   
   public void setRootChallengeRules(RobotChallengeRules robotChallengeRules)
   {
      this.robotChallengeRules = robotChallengeRules;
      this.collisionProcessor.setRobotChallengeRules(robotChallengeRules);
   }

   public FoodList01 getFoodList()
   {
      return foodList;
   }

   public PredatorList01 getPredatorList()
   {
      return predatorList;
   }

   public FlagList getFlagList()
   {
      return flagList;
   }
   
   public ArrayList<LineSegment2D> getWalls()
   {
      Point2D corner01 = new Point2D(0.0, 0.0);
      Point2D corner02 = new Point2D(0.0, yMax);
      Point2D corner03 = new Point2D(xMax, yMax);
      Point2D corner04 = new Point2D(xMax, 0.0);
      
      LineSegment2D wall1 = new LineSegment2D(corner01, corner02);
      LineSegment2D wall2 = new LineSegment2D(corner02, corner03);
      LineSegment2D wall3 = new LineSegment2D(corner03, corner04);
      LineSegment2D wall4 = new LineSegment2D(corner04, corner01);
      
      ArrayList<LineSegment2D> walls = new ArrayList<LineSegment2D>();
      walls.add(wall1);
      walls.add(wall2);
      walls.add(wall3);
      walls.add(wall4);
      
      return walls;
   }
   
   public Point2DBasics getIntersectionWithWall(Point2D position, Vector2D sensingVectorInWorld)
   {
      ArrayList<LineSegment2D> walls = getWalls();
      
      Line2D ray = new Line2D(position, sensingVectorInWorld);
      
      for (LineSegment2D wall : walls)
      {
         Point2DBasics intersection = wall.intersectionWith(ray);
         if (intersection != null)
         {
            Vector2D vectorToWall = new Vector2D(intersection);
            vectorToWall.sub(position);
            
            if (sensingVectorInWorld.dot(vectorToWall) > 0.0)
            {
               return intersection;
            }
         }
      }
      
      System.err.println("walls = " + walls);
      System.err.println("ray = " + ray);
      
      throw new RuntimeException("Shouldn't get here. Intersection with wall not found...");
//      return null;
   }
   

   public static void main(String[] args)
   {
      Random random = new Random();
      double xMax = 10.0;
      double yMax = 10.0;

      Robot01 robot = new Robot01(xMax, yMax);

      RobotChallenge01 robotChallenge01 = new RobotChallenge01("RobotChallenge01", robot, random, xMax, yMax);
      robotChallenge01.createSomeFood(10);

      Robot01Behavior simpleBehavior = new SimpleRobot01Behavior();
      ExperimentalBehavior01 behavior01 = new ExperimentalBehavior01();

      RobotChallengeRules rules = new RobotChallengeRules01(robot, robotChallenge01.getFoodList(), behavior01);
      robotChallenge01.setRootChallengeRules(rules);

      robotChallenge01.runSimulation();
   }

}
