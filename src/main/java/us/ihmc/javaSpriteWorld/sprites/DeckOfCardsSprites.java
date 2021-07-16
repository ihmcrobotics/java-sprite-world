package us.ihmc.javaSpriteWorld.sprites;

import java.util.ArrayList;
import java.util.EnumMap;

import javafx.scene.image.Image;
import javafx.scene.image.PixelReader;
import javafx.scene.image.WritableImage;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteCostume;
import us.ihmc.javaSpriteWorld.SpriteMouseListener;
import us.ihmc.javaSpriteWorld.SpriteWorld;
import us.ihmc.javaSpriteWorld.SpriteWorldMouseListener;
import us.ihmc.javaSpriteWorld.SpriteWorldViewer;
import us.ihmc.javaSpriteWorld.SpriteWorldViewerUsingSwing;

public class DeckOfCardsSprites
{
   private final EnumMap<DeckOfCardsNumber, EnumMap<DeckOfCardsSuit, Sprite>> sprites;

   public enum DeckOfCardsNumber
   {
      Ace, Two, Three, Four, Five, Six, Seven, Eight, Nine, Ten, Jack, Queen, King;
   }

   public enum DeckOfCardsSuit
   {
      Hearts, Diamonds, Clubs, Spades;
   }

   public DeckOfCardsSprites()
   {
      this(0.5, 0.5, 1.0);
   }

   public DeckOfCardsSprites(double xReferencePercent, double yReferencePercent, double height)
   {
      Image image = SpriteCostume.createImageFromFile("sampleImages/cards/playingCards.png");
      PixelReader reader = image.getPixelReader();

      sprites = new EnumMap<DeckOfCardsNumber, EnumMap<DeckOfCardsSuit, Sprite>>(DeckOfCardsNumber.class);

      for (int i = 0; i < 13; i++)
      {
         DeckOfCardsNumber number = DeckOfCardsNumber.values()[i];

         sprites.put(number, new EnumMap<DeckOfCardsSuit, Sprite>(DeckOfCardsSuit.class));
         for (int j = 0; j < 4; j++)
         {
            DeckOfCardsSuit suit = DeckOfCardsSuit.values()[j];

            WritableImage writeableImage = new WritableImage(reader, 12 + 153 * i, 12 + 223 * j, 141, 211);
            //      ImageView imageView = new ImageView(image);

            SpriteCostume costume = new SpriteCostume(writeableImage); //imageView.getImage());

            costume.setXReferencePercent(xReferencePercent);
            costume.setYReferencePercent(yReferencePercent);

            Sprite sprite = new Sprite(number + " Of " + suit);
            sprite.addCostume(costume);

            sprite.setHeightPreserveScale(height, 0);
            sprites.get(number).put(suit, sprite);
         }
      }
   }

   public ArrayList<Sprite> getAllCards()
   {
      ArrayList<Sprite> cards = new ArrayList<Sprite>();

      for (DeckOfCardsNumber number : DeckOfCardsNumber.values())
      {
         for (DeckOfCardsSuit suit : DeckOfCardsSuit.values())
         {
            cards.add(getCard(number, suit));
         }
      }

      return cards;
   }

   public Sprite getCard(DeckOfCardsNumber number, DeckOfCardsSuit suit)
   {
      return sprites.get(number).get(suit);
   }

   public static void main(String[] args)
   {
      SpriteWorldViewerUsingSwing viewer = new SpriteWorldViewerUsingSwing("Deck Viewer");
      SpriteWorld spriteWorld = new SpriteWorld();

      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setRightBorderX(4.5);

      spriteWorld.setTopBorderY(0.0);
      spriteWorld.setBottomBorderY(4.5);

      DeckOfCardsSprites deckOfCards = new DeckOfCardsSprites();

      double y = 0.75;
      for (DeckOfCardsSprites.DeckOfCardsSuit suit : DeckOfCardsSprites.DeckOfCardsSuit.values())
      {
         double x = 0.5;
         double rotation = -20.0;

         for (DeckOfCardsSprites.DeckOfCardsNumber number : DeckOfCardsSprites.DeckOfCardsNumber.values())
         {
            Sprite card = deckOfCards.getCard(number, suit);
            spriteWorld.addSprite(card);
            card.setX(x);
            card.setY(y + 6.0 - 6.0 * Math.cos(Math.PI * rotation / 180.0));
            card.setRotationInDegrees(rotation);

            x += 0.3;
            rotation += 4.0;
         }
         y += 0.5;
      }

      viewer.setSpriteWorld(spriteWorld);

      viewer.createAndDisplayWindow();
      viewer.update();

      SpriteMouseListener spriteMouseListener = new SpriteMouseListener()
      {

         @Override
         public void spriteReleased(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld)
         {
         }

         @Override
         public void spritePressed(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld)
         {
//            spriteWorld.moveSpriteForward(sprite);
//            spriteWorld.moveSpriteToFront(sprite);
//            viewer.update();
         }

         @Override
         public void spriteDragged(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld)
         {
            spriteWorld.moveSpriteToFront(sprite);
            sprite.setX(xWorld);
            sprite.setY(yWorld);
            viewer.update();
         }

         @Override
         public void spriteClicked(SpriteWorldViewer viewer, Sprite sprite, double xWorld, double yWorld, int clickCount)
         {
            if (clickCount > 1)
            {
               spriteWorld.moveSpriteBackward(sprite);
               spriteWorld.moveSpriteBackward(sprite);
               viewer.update();
            }
            else
            {
               spriteWorld.moveSpriteForward(sprite);
               viewer.update();
            }
         }
      };

      for (DeckOfCardsSprites.DeckOfCardsSuit suit : DeckOfCardsSprites.DeckOfCardsSuit.values())
      {
         for (DeckOfCardsSprites.DeckOfCardsNumber number : DeckOfCardsSprites.DeckOfCardsNumber.values())
         {
            Sprite card = deckOfCards.getCard(number, suit);
            card.attachSpriteMouseListener(spriteMouseListener);
         }
      }

      SpriteWorldMouseListener spriteWorldMouseListener = new SpriteWorldMouseListener()
      {

         @Override
         public void mouseReleasedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
         }

         @Override
         public void mousePressedInWorld(SpriteWorldViewer viewer, SpriteWorld spriteWorld, double xWorld, double yWorld)
         {
            System.out.println(xWorld + ", " + yWorld);
            //            aceOfHearts.setX(xWorld);
            //            aceOfHearts.setY(yWorld);

            viewer.update();
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

}
