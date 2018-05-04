package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

import org.junit.Test;

public class SpriteWorldViewerUsingJavaFXTest
{

   @Test
   public void testSimpleSpriteWorldViewer() throws InterruptedException
   {
      SpriteWorldViewerUsingSwing viewerOne = new SpriteWorldViewerUsingSwing("Test Using Swing");
      //      SpriteWorldViewerUsingSwing viewerTwo = new SpriteWorldViewerUsingSwing("Test Using Swing");

      //      SpriteWorldViewerUsingJavaFX viewerOne = new SpriteWorldViewerUsingJavaFX("Test Using JavaFX");
      SpriteWorldViewerUsingJavaFX viewerTwo = new SpriteWorldViewerUsingJavaFX("Test Using JavaFX");

      viewerOne.setPreferredSizeInPixels(300, 300);
      viewerOne.setResizable(false);

      viewerTwo.setPreferredSizeInPixels(300, 300);
      viewerTwo.setResizable(false);

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setTopBorderY(0.0);
      spriteWorld.setRightBorderX(300.0);
      spriteWorld.setBottomBorderY(300.0);

      Sprite crossHairs = new Sprite("CrossHairs");

      crossHairs.addCostume(SampleSpriteCostumes.getCrossHairs());
      crossHairs.setWidth(300.0);
      crossHairs.setHeight(300.0);

      crossHairs.setX(150.0);
      crossHairs.setY(150.0);

      crossHairs.setRotationInDegrees(5.0);

      spriteWorld.addSprite(crossHairs);

      viewerOne.setSpriteWorld(spriteWorld);
      viewerOne.createAndDisplayWindow();

      viewerTwo.setSpriteWorld(spriteWorld);
      viewerTwo.createAndDisplayWindow();

      double rotation = 5.0;

      while (true)
      {
         rotation = rotation + 5.0;
         crossHairs.setRotationInDegrees(rotation);
         viewerOne.update();
         Thread.sleep(100L);

         rotation = rotation + 5.0;
         crossHairs.setRotationInDegrees(rotation);
         viewerTwo.update();
         Thread.sleep(100L);

      }
   }

   @Test
   public void testSpriteWorldViewerTwo() throws InterruptedException
   {
      SpriteWorldViewerUsingSwing viewerOne = new SpriteWorldViewerUsingSwing("Test Using Swing");
      SpriteWorldViewerUsingJavaFX viewerTwo = new SpriteWorldViewerUsingJavaFX("Test Using JavaFX");

      viewerOne.setPreferredSizeInPixels(1200, 300);
      viewerOne.setResizable(false);

      viewerTwo.setPreferredSizeInPixels(1200, 300);
      viewerTwo.setResizable(false);

      SpriteWorld spriteWorld = new SpriteWorld();

      spriteWorld.setLeftBorderX(50.0);
      spriteWorld.setTopBorderY(100.0);
      spriteWorld.setRightBorderX(150.0);
      spriteWorld.setBottomBorderY(0.0);

      //      spriteWorld.setLeftBorderX(-50.0);
      //      spriteWorld.setTopBorderY(125.0);
      //      spriteWorld.setRightBorderX(50.0);
      //      spriteWorld.setBottomBorderY(25.0);

      Sprite crossHairs = new Sprite("CrossHairs");

      crossHairs.addCostume(SampleSpriteCostumes.getCrossHairs());
      crossHairs.setWidth(200.0);
      crossHairs.setHeight(200.0);

      crossHairs.setReflectX(true);

      //      crossHairs.setX(0.0);
      //      crossHairs.setY(0.0);

      crossHairs.setX(100.0);
      crossHairs.setY(50.0);

      //      crossHairs.setRotationInDegrees(0.0);
      crossHairs.setRotationInDegrees(30.0);

      double imageWidthPixels = crossHairs.getCostume().getImageWidthPixels();
      double imageHeightPixels = crossHairs.getCostume().getImageHeightPixels();

      //      System.out.println("imageWidthPixels = " + imageWidthPixels);
      //      System.out.println("imageHeightPixels = " + imageHeightPixels);

      spriteWorld.addSprite(crossHairs);

      viewerOne.setSpriteWorld(spriteWorld);
      viewerOne.createAndDisplayWindow();

      viewerTwo.setSpriteWorld(spriteWorld);
      viewerTwo.createAndDisplayWindow();

      while (true)
      {
         crossHairs.addRotationInRadians(0.01);

         viewerOne.update();
         viewerTwo.update();

         Thread.sleep(100L);
      }
   }

   @Test
   public void testSpriteWorldViewerThree() throws InterruptedException
   {
      double atmosphereHeight = 100.0;
      double worldWidth = 200.0;
      double x = 100.0;
      double y = 50.0;

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setTopBorderY(atmosphereHeight);
      spriteWorld.setBottomBorderY(0.0);
      spriteWorld.setLeftBorderX(-worldWidth / 2.0);
      spriteWorld.setRightBorderX(worldWidth / 2.0);

      Sprite crossHairs = SampleSprites.createCrossHairs();
      crossHairs.setWidth(worldWidth / 10.0);
      crossHairs.setHeight(atmosphereHeight / 10.0);
      crossHairs.setX(x);
      crossHairs.setY(y);
      spriteWorld.addSprite(crossHairs);

      int preferredHeight = 800;
      int preferredWidth = (int) (preferredHeight * worldWidth / atmosphereHeight);

      //      System.out.println("preferredHeight = " + preferredHeight);
      //      System.out.println("preferredWidth = " + preferredWidth);

      SpriteWorldViewer spriteWorldViewerOne = spriteWorld.createAndDisplaySpriteWorldViewerUsingJavaFX("Test", spriteWorld, preferredWidth, preferredHeight);
      SpriteWorldViewer spriteWorldViewerTwo = spriteWorld.createAndDisplaySpriteWorldViewerUsingSwing("Test", spriteWorld, preferredWidth, preferredHeight);

      spriteWorldViewerOne.update();
      spriteWorldViewerTwo.update();

      while (true)
      {
         Thread.sleep(100L);
      }
   }

   @Test
   public void testCostumeReference() throws InterruptedException
   {
      double atmosphereHeight = 100.0;
      double worldWidth = 200.0;
      double x = 0.0;
      double y = 50.0;

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setTopBorderY(atmosphereHeight);
      spriteWorld.setBottomBorderY(0.0);
      spriteWorld.setLeftBorderX(-worldWidth / 2.0);
      spriteWorld.setRightBorderX(worldWidth / 2.0);

      Sprite crossHairsOne = SampleSprites.createCrossHairs();
      crossHairsOne.setWidth(worldWidth / 10.0);
      crossHairsOne.setHeight(atmosphereHeight / 10.0);
      crossHairsOne.setX(x);
      crossHairsOne.setY(y);
      spriteWorld.addSprite(crossHairsOne);

      Sprite crossHairsTwo = SampleSprites.createCrossHairs();
      crossHairsTwo.setWidth(worldWidth / 10.0);
      crossHairsTwo.setHeight(atmosphereHeight / 10.0);
      crossHairsTwo.setX(x);
      crossHairsTwo.setY(y);
      crossHairsTwo.setReflectY(true);
      spriteWorld.addSprite(crossHairsTwo);

      int preferredHeight = 800;
      int preferredWidth = (int) (preferredHeight * worldWidth / atmosphereHeight);

      //      System.out.println("preferredHeight = " + preferredHeight);
      //      System.out.println("preferredWidth = " + preferredWidth);

      SpriteWorldViewer spriteWorldViewerOne = spriteWorld.createAndDisplaySpriteWorldViewerUsingJavaFX("Test", spriteWorld, preferredWidth, preferredHeight);
      SpriteWorldViewer spriteWorldViewerTwo = spriteWorld.createAndDisplaySpriteWorldViewerUsingSwing("Test", spriteWorld, preferredWidth, preferredHeight);

      double xReferencePercent = 0.5;
      double yReferencePercent = 0.5;

      while (true)
      {
         spriteWorldViewerOne.update();
         spriteWorldViewerTwo.update();

         //         xReferencePercent+= 0.1;
         yReferencePercent += 0.1;
         if (xReferencePercent > 1.0)
            xReferencePercent = 0.0;
         if (yReferencePercent > 1.0)
            yReferencePercent = 0.0;

         crossHairsTwo.getCostume().setXReferencePercent(xReferencePercent);
         crossHairsTwo.getCostume().setYReferencePercent(yReferencePercent);

         Thread.sleep(100L);
      }
   }

   @Test
   public void testRocketSprites() throws InterruptedException
   {
      double atmosphereHeight = 100.0;
      double worldWidth = 200.0;
      double rocketHeight = 20.0;
      double rocketWidth = 10.0;

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setTopBorderY(atmosphereHeight);
      spriteWorld.setBottomBorderY(0.0);
      spriteWorld.setLeftBorderX(-worldWidth / 2.0);
      spriteWorld.setRightBorderX(worldWidth / 2.0);

      double x = 0.0;
      double y = 40.0;

      Sprite crossHairs = SampleSprites.createCrossHairs();
      crossHairs.setWidth(worldWidth / 10.0);
      crossHairs.setHeight(worldWidth / 10.0);
      crossHairs.setX(x);
      crossHairs.setY(y);
      spriteWorld.addSprite(crossHairs);

      Sprite rocketSprite = SampleSprites.createRocketOne();
      rocketSprite.setHeight(rocketHeight);
      rocketSprite.setWidth(rocketWidth);
      rocketSprite.setX(x);
      rocketSprite.setY(y);
      rocketSprite.setReflectX(true);
      rocketSprite.setReflectY(true);

      spriteWorld.addSprite(rocketSprite);

      Sprite rocketFlameSprite = SampleSprites.createRocketFlameOne();
      rocketFlameSprite.setHeight(rocketHeight / 4.0);
      rocketFlameSprite.setWidth(rocketHeight / 4.0);
      rocketFlameSprite.setReflectX(true);
      rocketFlameSprite.setReflectY(true);
      spriteWorld.addSprite(rocketFlameSprite);

      int preferredHeight = 800;
      int preferredWidth = 3 * (int) (preferredHeight * worldWidth / atmosphereHeight);

      //      System.out.println("preferredHeight = " + preferredHeight);
      //      System.out.println("preferredWidth = " + preferredWidth);

      SpriteWorldViewer spriteWorldViewerOne = spriteWorld.createAndDisplaySpriteWorldViewerUsingJavaFX("Test Rocket", spriteWorld, preferredWidth,
                                                                                                        preferredHeight);
      SpriteWorldViewer spriteWorldViewerTwo = spriteWorld.createAndDisplaySpriteWorldViewerUsingSwing("Test Rocket", spriteWorld, preferredWidth,
                                                                                                       preferredHeight);

      double rocketFlameAngle = 0.0;
      double rocketFlameHeight = 12.0;

      double rocketAngle = 0.0;

      while (true)
      {
         spriteWorldViewerOne.update();
         spriteWorldViewerTwo.update();

         //         rocketSprite.addX(0.1);
         //         rocketSprite.addY(1.0);
         rocketSprite.setRotationInRadians(rocketAngle);
         rocketAngle += 0.1;

         double distance = -6.5;

         rocketFlameSprite.setX(rocketSprite.getX() - distance * Math.sin(rocketSprite.getRotationInRadians()));
         rocketFlameSprite.setY(rocketSprite.getY() + distance * Math.cos(rocketSprite.getRotationInRadians()));

         rocketFlameSprite.setRotationInRadians(rocketAngle + rocketFlameAngle);
         rocketFlameAngle -= 0.037;
         //         rocketFlameHeight += 0.1;

         crossHairs.setX(rocketFlameSprite.getX());
         crossHairs.setY(rocketFlameSprite.getY());

         rocketFlameSprite.setHeightPreserveScale(rocketFlameHeight, 0);

         Thread.sleep(100L);

      }
   }

   @Test
   public void testSimpleDragging() throws InterruptedException
   {
      SpriteWorldViewerUsingSwing viewerOne = new SpriteWorldViewerUsingSwing("Test Using Swing");
      SpriteWorldViewerUsingJavaFX viewerTwo = new SpriteWorldViewerUsingJavaFX("Test Using JavaFX");

      //      SpriteWorldViewer viewer = new SpriteWorldViewer("Test");

      viewerOne.setPreferredSizeInPixels(500, 900);
      viewerOne.setResizable(false);

      viewerTwo.setPreferredSizeInPixels(500, 900);
      viewerTwo.setResizable(false);

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(-0.3);
      spriteWorld.setTopBorderY(0.6);
      spriteWorld.setRightBorderX(2.1);
      spriteWorld.setBottomBorderY(-0.11);

      Sprite blackDie = SampleSprites.createSixSidedBlackPipsOnWhiteDie();
      blackDie.setHeightPreserveScale(0.1, 0);
      spriteWorld.addSprite(blackDie);

      double blackDieX = 0.2;
      double blackDieY = 0.5;

      blackDie.setX(blackDieX);
      blackDie.setY(blackDieY);

      blackDie.attachSpriteMouseListener(new DragAlongSpriteMouseListener(blackDie));

      Sprite redDie = SampleSprites.createSixSidedRedPipsOnWhiteDie();
      redDie.setHeightPreserveScale(0.1, 0);
      spriteWorld.addSprite(redDie);

      double redDieX = 0.8;
      double redkDieY = 0.5;

      redDie.setX(redDieX);
      redDie.setY(redkDieY);

      redDie.attachSpriteMouseListener(new DragAlongSpriteMouseListener(redDie));

      viewerOne.setSpriteWorld(spriteWorld);
      viewerOne.createAndDisplayWindow();

      viewerTwo.setSpriteWorld(spriteWorld);
      viewerTwo.createAndDisplayWindow();
      while (true)
      {
         blackDie.nextCostume();
         redDie.previousCostume();

         viewerOne.update();
         viewerTwo.update();
         Thread.sleep(100);
      }
   }

   @Test
   public void testSpriteWorldViewer() throws InterruptedException
   {
      SpriteWorldViewerUsingSwing viewerOne = new SpriteWorldViewerUsingSwing("Test Using Swing");
      SpriteWorldViewerUsingJavaFX viewerTwo = new SpriteWorldViewerUsingJavaFX("Test Using JavaFX");

      //      SpriteWorldViewer viewer = new SpriteWorldViewer("Test");

      viewerOne.setPreferredSizeInPixels(1000, 500);
      viewerOne.setResizable(false);

      viewerTwo.setPreferredSizeInPixels(1000, 500);
      viewerTwo.setResizable(false);

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setTopBorderY(1.0);
      spriteWorld.setRightBorderX(2.0);
      spriteWorld.setBottomBorderY(0.0);

      SpriteStage stage = new SpriteStage("Test Stage");
      StageBackdrop backgammonBoardBackdrop = SampleStageBackdrops.getBackgammonBoard();
      backgammonBoardBackdrop.setXReferencePercent(0.5);
      backgammonBoardBackdrop.setYReferencePercent(0.5);

      stage.addBackdrop(backgammonBoardBackdrop);
      spriteWorld.setStage(stage, true);

      Sprite rocket = SampleSprites.createRocketOne();
      rocket.setWidth(0.1);
      rocket.setHeight(0.2);
      rocket.setX(1.0);
      rocket.setY(0.5);
      spriteWorld.addSprite(rocket);

      Sprite whiteKing = new Sprite("White King");
      SpriteCostume whiteChessKingCostume = SampleSpriteCostumes.getWhiteChessKing();
      whiteChessKingCostume.setXReferencePercent(0.5);
      whiteChessKingCostume.setYReferencePercent(0.5);
      whiteKing.setReflectX(true);
      whiteKing.setReflectY(true);
      whiteKing.addCostume(whiteChessKingCostume);
      whiteKing.setWidth(0.25);
      whiteKing.setHeight(0.125);
      whiteKing.setX(1.0);
      whiteKing.setY(0.5);

      spriteWorld.addSprite(whiteKing);

      Sprite crossHairs = new Sprite("CrossHairs");
      crossHairs.addCostume(SampleSpriteCostumes.getCrossHairs());
      crossHairs.setWidth(0.5);
      crossHairs.setHeight(0.5);
      crossHairs.setX(1.0);
      crossHairs.setY(0.5);
      crossHairs.setRotationInRadians(0.0);

      spriteWorld.addSprite(crossHairs);

      Sprite crossHairs2 = new Sprite("SpinningCrossHairs");
      crossHairs2.addCostume(SampleSpriteCostumes.getCrossHairs());
      crossHairs2.setWidth(0.5);
      crossHairs2.setHeight(0.5);
      crossHairs2.setX(1.0);
      crossHairs2.setY(0.5);
      crossHairs2.setRotationInRadians(0.8);

      spriteWorld.addSprite(crossHairs2);

      viewerOne.setSpriteWorld(spriteWorld);
      viewerOne.createAndDisplayWindow();

      viewerTwo.setSpriteWorld(spriteWorld);
      viewerTwo.createAndDisplayWindow();

      Sprite blackDie = SampleSprites.createSixSidedBlackPipsOnWhiteDie();
      blackDie.setHeightPreserveScale(0.1, 0);
      spriteWorld.addSprite(blackDie);

      double blackDieX = 0.2;
      double blackDieY = 0.5;

      blackDie.setX(blackDieX);
      blackDie.setY(blackDieY);

      blackDie.attachSpriteMouseListener(new DragAlongSpriteMouseListener(blackDie));

      Sprite redDie = SampleSprites.createSixSidedRedPipsOnWhiteDie();
      redDie.setHeightPreserveScale(0.1, 0);
      spriteWorld.addSprite(redDie);

      double redDieX = 1.8;
      double redkDieY = 0.5;

      redDie.setX(redDieX);
      redDie.setY(redkDieY);

      redDie.attachSpriteMouseListener(new DragAlongSpriteMouseListener(redDie));

      double rotation = 0.0;
      double kingHeight = whiteKing.getHeight();

      while (true)
      {
         crossHairs.setRotationInRadians(rotation);
         rotation += 0.2;

         whiteKing.setHeightPreserveScale(kingHeight, 0);
         kingHeight = kingHeight * 1.1;
         if (kingHeight > spriteWorld.getHeight())
            kingHeight = 0.01;

         blackDie.nextCostume();
         redDie.previousCostume();

         viewerOne.update();
         viewerTwo.update();
         Thread.sleep(100);
      }
   }

   @Test
   public void testMultipleSpriteWorldViewers() throws InterruptedException
   {
      int numberOfSwingViewers = 6;
      int numberOfJavaFXViewers = 6;

      ArrayList<SpriteWorldViewer> viewers = new ArrayList<>();

      for (int i = 0; i < numberOfSwingViewers; i++)
      {
         SpriteWorldViewer swingViewer = new SpriteWorldViewerUsingSwing("Test Using Swing");
         swingViewer.setLocationOnScreen(i * 250, i*150);
         viewers.add(swingViewer);
      }

      for (int i = 0; i < numberOfJavaFXViewers; i++)
      {
         SpriteWorldViewer javaFXViewer = new SpriteWorldViewerUsingJavaFX("Test Using JavaFX");
         javaFXViewer.setLocationOnScreen(500 + i * 250, i*150);
         viewers.add(javaFXViewer);
      }

      for (SpriteWorldViewer viewer : viewers)
      {
         viewer.setPreferredSizeInPixels(300, 300);
         viewer.setResizable(false);
      }

      SpriteWorld spriteWorldOne = new SpriteWorld();
      spriteWorldOne.setLeftBorderX(0.0);
      spriteWorldOne.setTopBorderY(0.0);
      spriteWorldOne.setRightBorderX(300.0);
      spriteWorldOne.setBottomBorderY(300.0);
      
      SpriteWorld spriteWorldTwo = new SpriteWorld();
      spriteWorldTwo.setLeftBorderX(0.0);
      spriteWorldTwo.setTopBorderY(0.0);
      spriteWorldTwo.setRightBorderX(300.0);
      spriteWorldTwo.setBottomBorderY(300.0);

      Sprite crossHairsOne = createCrossHairs();
      Sprite crossHairsTwo = createCrossHairs();

      crossHairsTwo.setHeight(0.5 * crossHairsTwo.getHeight());
      crossHairsTwo.setWidth(0.5 * crossHairsTwo.getWidth());
      
      spriteWorldOne.addSprite(crossHairsOne);
      spriteWorldTwo.addSprite(crossHairsTwo);

      int count = 0;
      for (SpriteWorldViewer viewer : viewers)
      {
         if (count % 2 == 0)
         {
            viewer.setSpriteWorld(spriteWorldOne);
         }
         else
         {
            viewer.setSpriteWorld(spriteWorldTwo);
         }
         
         viewer.createAndDisplayWindow();
         count++;
      }

      double rotation = 5.0;

      while (true)
      {
         for (SpriteWorldViewer viewer : viewers)
         {
            Thread.sleep(10L);
            rotation = rotation + 1.0;
            crossHairsOne.setRotationInDegrees(rotation);
            crossHairsTwo.setRotationInDegrees(-rotation);
            viewer.update();
         }
      }
   }
   
   private Sprite createCrossHairs()
   {
      Sprite crossHairs = new Sprite("CrossHairs");

      crossHairs.addCostume(SampleSpriteCostumes.getCrossHairs());
      crossHairs.setWidth(300.0);
      crossHairs.setHeight(300.0);

      crossHairs.setX(150.0);
      crossHairs.setY(150.0);

      crossHairs.setRotationInDegrees(5.0);
      
      return crossHairs;
   }

   private class DragAlongSpriteMouseListener implements SpriteMouseListener
   {
      private final Sprite sprite;

      public DragAlongSpriteMouseListener(Sprite sprite)
      {
         this.sprite = sprite;
      }

      @Override
      public void spriteClicked(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld, int clickCount)
      {
         System.out.println("Sprite " + sprite.getName() + " clicked at world " + xWorld + ", " + yWorld + ". clickCount = " + clickCount);
      }

      @Override
      public void spritePressed(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld) //, MouseEvent mouseEvent)
      {
         System.out.println("Sprite " + sprite.getName() + " pressed at world " + xWorld + ", " + yWorld);
      }

      @Override
      public void spriteReleased(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld) //, MouseEvent mouseEvent)
      {
         System.out.println("Sprite " + sprite.getName() + " released at world " + xWorld + ", " + yWorld);
      }

      @Override
      public void spriteDragged(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld) //, MouseEvent mouseEvent)
      {
         System.out.println("Sprite " + sprite.getName() + " dragged at world " + xWorld + ", " + yWorld);
         sprite.setX(xWorld);
         sprite.setY(yWorld);
      }

   }

}
