package us.ihmc.javaSpriteWorld;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import javafx.scene.input.MouseEvent;
import us.ihmc.javaSpriteWorld.SampleSpriteCostumes;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.SampleStageBackdrops;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCollisionGroup;
import us.ihmc.javaSpriteWorld.SpriteCollisionListener;
import us.ihmc.javaSpriteWorld.SpriteCostume;
import us.ihmc.javaSpriteWorld.SpriteMouseListener;
import us.ihmc.javaSpriteWorld.SpriteStage;
import us.ihmc.javaSpriteWorld.SpriteWorld;
import us.ihmc.javaSpriteWorld.SpriteWorldViewerUsingSwing;
import us.ihmc.javaSpriteWorld.StageBackdrop;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

@Tag("gui")
public class SpriteWorldViewerTest
{

   @Test
   public void testSpriteWorldViewer()
   {
      SpriteWorldViewerUsingSwing viewer = new SpriteWorldViewerUsingSwing("Test");
      
      viewer.setPreferredSizeInPixels(1000, 500); 
      viewer.setResizable(false);
      
      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setTopBorderY(0.0);
      spriteWorld.setRightBorderX(2.0);
      spriteWorld.setBottomBorderY(1.0);
      
      SpriteStage stage = new SpriteStage("Test Stage");
      StageBackdrop backgammonBoardBackdrop = SampleStageBackdrops.getBackgammonBoard();
      backgammonBoardBackdrop.setXReferencePercent(0.5);
      backgammonBoardBackdrop.setYReferencePercent(0.5);
      
      
      stage.addBackdrop(backgammonBoardBackdrop);
      spriteWorld.setStage(stage, true);

      SpriteCollisionGroup collisionGroup = new SpriteCollisionGroup();
      
      Sprite rocket = SampleSprites.createRocketOne();
      rocket.setWidth(0.1);
      rocket.setHeight(0.2);
      rocket.setX(1.0);
      rocket.setY(0.5);
      spriteWorld.addSprite(rocket);
//      collisionGroup.addSprite(rocket);
      
      Sprite whiteKing = new Sprite("White King");
      SpriteCostume whiteChessKingCostume = SampleSpriteCostumes.getWhiteChessKing();
      whiteChessKingCostume.setXReferencePercent(0.5);
      whiteChessKingCostume.setYReferencePercent(0.5);
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
      
      viewer.setSpriteWorld(spriteWorld);
      viewer.createAndDisplayWindow();
      
      Sprite blackDie = SampleSprites.createSixSidedBlackPipsOnWhiteDie();
      blackDie.setHeightPreserveScale(0.1, 0);
      
      
      ConvexPolygon collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.05, -0.05), new Vector(0.1, 0.1));
      blackDie.addCollisionPolygon(collisionPolygon);
      spriteWorld.addSprite(blackDie);
      collisionGroup.addSprite(blackDie);

      double blackDieX = 0.2;
      double blackDieY = 0.5;
      
      blackDie.setX(blackDieX);
      blackDie.setY(blackDieY);
      
      blackDie.attachSpriteMouseListener(new DragAlongSpriteMouseListener(blackDie));
      
      Sprite redDie = SampleSprites.createSixSidedRedPipsOnWhiteDie();
      redDie.setHeightPreserveScale(0.1, 0);
      
      collisionPolygon = ConvexPolygon.createRectangle(new Point(-0.05, -0.05), new Vector(0.1, 0.1));
      redDie.addCollisionPolygon(collisionPolygon);
      
      spriteWorld.addSprite(redDie);
      collisionGroup.addSprite(redDie);

      double redDieX = 1.8;
      double redkDieY = 0.5;
      
      redDie.setX(redDieX);
      redDie.setY(redkDieY);
      
      redDie.attachSpriteMouseListener(new DragAlongSpriteMouseListener(redDie));
      
      double rotation = 0.0;
      double kingHeight = whiteKing.getHeight();
      
      collisionGroup.addSpriteCollisionListener(new SpriteCollisionListener()
      {
         
         @Override
         public void spritesAreColliding(Sprite spriteOne, Sprite spriteTwo)
         {
            System.out.println(spriteOne.getName() + " collided with " + spriteTwo.getName() + "!");
         }
      });
      
      spriteWorld.addSpriteCollisionGroup(collisionGroup);
      
      while(true)
      {
         try
         {
            crossHairs.setRotationInRadians(rotation);
            rotation += 0.2;
            
            whiteKing.setHeightPreserveScale(kingHeight, 0);
            kingHeight = kingHeight * 1.1;
            if (kingHeight > spriteWorld.getHeight()) kingHeight = 0.01;
            
            blackDie.nextCostume();
            redDie.previousCostume();

            viewer.update();
            spriteWorld.checkSpriteCollisions();
            
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
         }
      }
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
	   public void spritePressed(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld)
	   {
		   System.out.println("Sprite " + sprite.getName() + " pressed at world " + xWorld + ", " + yWorld);
	   }

	   @Override
	   public void spriteReleased(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld)
	   {
		   System.out.println("Sprite " + sprite.getName() + " released at world " + xWorld + ", " + yWorld);
	   }

	   @Override
	   public void spriteDragged(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld)
	   {
//		   System.out.println("Sprite " + sprite.getName() + " dragged at world " + xWorld + ", " + yWorld);
		   sprite.setX(xWorld);
		   sprite.setY(yWorld);
	   }

   }

}
