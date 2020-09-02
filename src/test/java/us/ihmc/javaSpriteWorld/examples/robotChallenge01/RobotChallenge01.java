package us.ihmc.javaSpriteWorld.examples.robotChallenge01;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tuple2D.Vector2D;
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
   private final Robot01 robot;
   private final FoodList01 foodList = new FoodList01();

   private Robot01Behavior robot01Behavior;
   private SpriteCollisionGroup collisionGroup;

   private boolean mousePressed;
   private double mousePressedX, mousePressedY;

   public RobotChallenge01(Random random, double xMax, double yMax)
   {
      this.random = random;
      this.xMax = xMax;
      this.yMax = yMax;

      viewer = new SpriteWorldViewerUsingSwing("RobotChallenge01");

      viewer.setPreferredSizeInPixels(1000, 1000);
      viewer.setResizable(false);

      spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setBottomBorderY(0.0);
      spriteWorld.setRightBorderX(xMax);
      spriteWorld.setTopBorderY(yMax);

      stage = new SpriteStage("Robot Challenge01 Stage");
      //      StageBackdrop backgammonBoardBackdrop = SampleStageBackdrops.getBackgammonBoard();
      //      backgammonBoardBackdrop.setXReferencePercent(0.5);
      //      backgammonBoardBackdrop.setYReferencePercent(0.5);

      //      stage.addBackdrop(backgammonBoardBackdrop);
      //      spriteWorld.setStage(stage, true);

      robot = new Robot01(xMax, yMax);
      spriteWorld.addSprite(robot.getSprite());

      collisionGroup = new SpriteCollisionGroup();
      collisionGroup.addSprite(robot.getSprite());

      viewer.setSpriteWorld(spriteWorld);
      viewer.createAndDisplayWindow();

      CollisionProcessor01 collisionProcessor = new CollisionProcessor01(robot, foodList, random, xMax, yMax, spriteWorld, collisionGroup);
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

   public void runSimulation()
   {
      double dt = 0.01;

      while (true)
      {
         if (robot01Behavior != null)
         {
            if (mousePressed)
            {
               robot01Behavior.senseMousePressed(mousePressedX, mousePressedY);
               mousePressed = false;
            }
            robot01Behavior.senseGlobalLocation(robot.getX(), robot.getY());

            ArrayList<Vector2D> locationOfAllFood = foodList.getLocationOfAllFood();
            robot01Behavior.senseFood(locationOfAllFood);

            double[] xyVelocity = robot01Behavior.getXYVelocity();

            robot.setXDot(xyVelocity[0]);
            robot.setYDot(xyVelocity[1]);
         }

         foodList.doDynamicsAndUpdateSprites(dt);

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

   public void setRobot01Behavior(Robot01Behavior robot01Behavior)
   {
      this.robot01Behavior = robot01Behavior;
   }

   public static void main(String[] args)
   {
      Random random = new Random();
      RobotChallenge01 robotChallenge01 = new RobotChallenge01(random, 10.0, 10.0);
      robotChallenge01.createSomeFood(10);
      Robot01Behavior simpleBehavior = new SimpleRobot01Behavior();
      robotChallenge01.setRobot01Behavior(simpleBehavior);
      robotChallenge01.runSimulation();
   }
}
