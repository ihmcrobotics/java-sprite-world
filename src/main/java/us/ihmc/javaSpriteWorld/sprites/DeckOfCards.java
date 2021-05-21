package us.ihmc.javaSpriteWorld.sprites;

import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCostume;
import us.ihmc.javaSpriteWorld.SpriteWorld;
import us.ihmc.javaSpriteWorld.SpriteWorldViewerUsingSwing;

public class DeckOfCards
{
   private final SpriteCostume aceOfHeartsCostume;
   private final Sprite aceOfHearts;
   
   public DeckOfCards()
   { 
      aceOfHeartsCostume = SpriteCostume.createFromFile("sampleImages/cards/playingCards.png");
      
      aceOfHeartsCostume.setXReferencePercent(0.1);
      aceOfHeartsCostume.setYReferencePercent(0.1);
      
      aceOfHeartsCostume.
      aceOfHearts = new Sprite("aceOfHearts");
      aceOfHearts.setHeightPreserveScale(2.0, 0);
      aceOfHearts.addCostume(aceOfHeartsCostume);
      
   }
   
   public Sprite getAceOfHearts()
   {
      return aceOfHearts;
   }
   
   
   public static void main(String[] args)
   {
      SpriteWorldViewerUsingSwing viewer = new SpriteWorldViewerUsingSwing("Deck Viewer");
      SpriteWorld spriteWorld = new SpriteWorld();

      spriteWorld.setLeftBorderX(-10.0);
      spriteWorld.setTopBorderY(10.0);
      spriteWorld.setRightBorderX(10.0);
      spriteWorld.setBottomBorderY(-10.0);
      
      DeckOfCards deckOfCards = new DeckOfCards();
      Sprite aceOfHearts = deckOfCards.getAceOfHearts();
      
      spriteWorld.addSprite(aceOfHearts);
      aceOfHearts.setX(1.0);
      aceOfHearts.setY(1.0);

      viewer.setSpriteWorld(spriteWorld);

      viewer.createAndDisplayWindow();

   }
     
   
   
}
