package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.Random;

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

      CollisionProcessor01 collisionProcessor = new CollisionProcessor01(robot, foodList, predatorList, random, xMax, yMax, spriteWorld, collisionGroup);
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

   public void setRootChallengeRules(RobotChallengeRules robotChallengeRules)
   {
      this.robotChallengeRules = robotChallengeRules;
   }

   public FoodList01 getFoodList()
   {
      return foodList;
   }

   public PredatorList01 getPredatorList()
   {
      return predatorList;
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
      RobotChallengeRules rules = new RobotChallengeRules01(robot, robotChallenge01.getFoodList(), simpleBehavior);

      robotChallenge01.setRootChallengeRules(rules);
      robotChallenge01.runSimulation();
   }

}
