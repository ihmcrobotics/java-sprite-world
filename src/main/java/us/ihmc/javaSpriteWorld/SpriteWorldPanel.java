package us.ihmc.javaSpriteWorld;

import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;

public class SpriteWorldPanel extends JPanel implements MouseMotionListener, MouseListener
{
   private static final long serialVersionUID = 4076112028926697537L;
   
   private final AffineTransform scaleForViewerPixelSize = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform scaleForWorldSize = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform translateForTopLeftCorner = AffineTransform.getTranslateInstance(0.0, 0.0);
   private final AffineTransform translateForSpritePosition = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform spriteRotation = AffineTransform.getRotateInstance(0.0);
   private final AffineTransform spriteReflection = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform scaleForSpriteSizeToImageSize = AffineTransform.getScaleInstance(1.0,  1.0);
   private final AffineTransform translateForImageCenter = AffineTransform.getTranslateInstance(0.0, 0.0);
   
   private final AffineTransform combinedTransform = new AffineTransform();

   private SpriteWorld spriteWorld;
   
   public SpriteWorldPanel(SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;
      
      this.addMouseListener(this);
      this.addMouseMotionListener(this);
   }
   
   @Override
   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);
      Graphics2D graphics = (Graphics2D) g;
      
      if (spriteWorld == null) return;
      
      SpriteStage stage = spriteWorld.getSpriteStage();
      
      double panelWidth = getWidth();
      double panelHeight = getHeight();
      
      if (stage != null)
      {
         StageBackdrop backdrop = stage.getBackdrop();
         if (backdrop != null)
         {
            Image image = backdrop.getImage();
            BufferedImage bufferedImage = SwingFXUtils.fromFXImage(image, null);
            graphics.drawImage(bufferedImage, 0, 0, ((int) panelWidth), ((int) panelHeight), null);	
         }
      }

      List<Sprite> sprites = spriteWorld.getSprites();

      for (int i=0; i<sprites.size(); i++)
      {
         Sprite sprite = sprites.get(i);
         if (sprite.isHidden()) continue;
         
         SpriteCostume currentCostume = sprite.getCostume();
         Image fxImage = currentCostume.getImage();
         
         BufferedImage image = SwingFXUtils.fromFXImage(fxImage, null);
         
         if (image == null) continue;
         
         double leftBorderX = spriteWorld.getLeftBorderX();
         double topBorderY = spriteWorld.getTopBorderY();
         double rightBorderX = spriteWorld.getRightBorderX();
         double bottomBorderY = spriteWorld.getBottomBorderY();

         double imageWidthPixels = image.getWidth();
         double imageHeightPixels = image.getHeight();

         double xReferencePercent = currentCostume.getXReferencePercent();
         double yReferencePercent = currentCostume.getYReferencePercent();

         double imageCenterX = imageWidthPixels * xReferencePercent;
         double imageCenterY = imageHeightPixels * yReferencePercent;

         scaleForViewerPixelSize.setToScale((double) panelWidth, ((double) panelHeight));
         
         scaleForWorldSize.setToScale(1.0/(rightBorderX - leftBorderX), 1.0/(bottomBorderY - topBorderY));
         translateForTopLeftCorner.setToTranslation(-leftBorderX, -topBorderY);
         
         translateForSpritePosition.setToTranslation(sprite.getX(), sprite.getY());
        
         spriteRotation.setToRotation(sprite.getRotationInRadians());
         
         double reflectX = 1.0;
         double reflectY = 1.0;
         
         if (sprite.getReflectX()) reflectX = -1.0;
         if (sprite.getReflectY()) reflectY = -1.0;
         
         spriteReflection.setToScale(reflectX, reflectY);
         
         scaleForSpriteSizeToImageSize.setToScale(sprite.getWidth() / imageWidthPixels, sprite.getHeight() / imageHeightPixels);
         translateForImageCenter.setToTranslation(-imageCenterX, -imageCenterY);
         
         combinedTransform.setToIdentity();

         combinedTransform.concatenate(scaleForViewerPixelSize);
         combinedTransform.concatenate(scaleForWorldSize);
         combinedTransform.concatenate(translateForTopLeftCorner);
         combinedTransform.concatenate(translateForSpritePosition);
         combinedTransform.concatenate(spriteRotation);
         combinedTransform.concatenate(spriteReflection);
         combinedTransform.concatenate(scaleForSpriteSizeToImageSize);
         combinedTransform.concatenate(translateForImageCenter);
         
         graphics.drawImage(image, combinedTransform, null);
      }
   }
   
   public double convertFromPanelPixelsToWorldX(int panelPixels)
   {
	   double panelWidth = getWidth();

	   double leftBorderX = spriteWorld.getLeftBorderX();
	   double rightBorderX = spriteWorld.getRightBorderX();
	   
	   return leftBorderX + ((double) panelPixels) / panelWidth * (rightBorderX - leftBorderX);
   }
   
   public double convertFromPanelPixelsToWorldY(int panelPixels)
   {
	   double panelHeight = getHeight();

	   double topBorderY = spriteWorld.getTopBorderY();
	   double bottomBorderY = spriteWorld.getBottomBorderY();
	   
	   return topBorderY + ((double) panelPixels) / panelHeight * (bottomBorderY - topBorderY);
   }

   public void setSpriteWorld(SpriteWorld spriteWorld) 
   {
	   this.spriteWorld = spriteWorld;	
   }

   private Sprite findTopMostSpriteWithMouseListenersAt(double worldX, double worldY)
   {
	   List<Sprite> sprites = spriteWorld.getSprites();
	   for (int i=sprites.size() - 1; i>=0; i--)
	   {
		   Sprite sprite = sprites.get(i);
		   
		   double spriteWidth = sprite.getWidth();
		   double spriteHeight = sprite.getHeight();
		   
		   double spriteX = sprite.getX();
		   double spriteY = sprite.getY();
		   
		   if ((worldX > spriteX - spriteWidth/2.0) && (worldX < spriteX + spriteWidth/2.0) && (worldY > spriteY - spriteHeight/2.0) && (worldY < spriteY + spriteHeight/2.0))
		   {
			   if (sprite.getSpriteMouseListeners().size() > 0) return sprite;
		   }
	   }
	   
	   return null;
   }
   
   private Sprite spriteCurrentlyBeingDragged = null;
   
   @Override
   public void mouseDragged(MouseEvent mouseEvent) 
   {	   
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
	   Sprite sprite = spriteCurrentlyBeingDragged;
//	   System.out.println("mouse dragged " + xWorld + ", " + yWorld + " sprite = " + sprite);
	   
	   if (sprite != null)
	   {
		   ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
		   
		   for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
		   {
			   spriteMouseListener.spriteDragged(sprite, xWorld, yWorld); //, mouseEvent);
		   }
	   }
	   else
	   {
	      ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
	      
	      for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
	         spriteWorldMouseListener.worldDragged(xWorld, yWorld);
         }
	   }
   }

   @Override
   public void mouseClicked(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
	   Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
//	   System.out.println("mouse clicked " + xWorld + ", " + yWorld + " sprite = " + sprite);
	   
	   javafx.scene.input.MouseEvent javaFXMouseEvent = null;
	   
	   if (sprite != null)
	   {
		   ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
		   
		   for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
		   {
			   spriteMouseListener.spriteClicked(sprite, xWorld, yWorld, javaFXMouseEvent);
		   }
	   }
	   else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.worldClicked(xWorld, yWorld, javaFXMouseEvent);
         }
      }
   }

   @Override
   public void mousePressed(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
	   Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
//	   System.out.println("mouse pressed " + xWorld + ", " + yWorld + " sprite = " + sprite);
	   
	   spriteCurrentlyBeingDragged = sprite;
	   
	   if (sprite != null)
	   {
		   ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
		   
		   for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
		   {
			   spriteMouseListener.spritePressed(sprite, xWorld, yWorld); //, mouseEvent);
		   }
	   }
	   else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.worldPressed(xWorld, yWorld);
         }
      }
	   
   }

   @Override
   public void mouseReleased(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
	   Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
//	   System.out.println("mouse released " + xWorld + ", " + yWorld + " sprite = " + sprite);
	   
	   spriteCurrentlyBeingDragged = null;
	   
	   if (sprite != null)
	   {
		   ArrayList<SpriteMouseListener> spriteMouseListeners = sprite.getSpriteMouseListeners();
		   
		   for (SpriteMouseListener spriteMouseListener : spriteMouseListeners)
		   {
			   spriteMouseListener.spriteReleased(sprite, xWorld, yWorld); //, mouseEvent);
		   }
	   }
	   else
      {
         ArrayList<SpriteWorldMouseListener> spriteWorldMouseListeners = spriteWorld.getSpriteWorldMouseListeners();
         
         for (SpriteWorldMouseListener spriteWorldMouseListener : spriteWorldMouseListeners)
         {
            spriteWorldMouseListener.worldReleased(xWorld, yWorld);
         }
      }
	   
   }
   
   @Override
   public void mouseMoved(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
	   Sprite sprite = findTopMostSpriteWithMouseListenersAt(xWorld, yWorld);
//	   System.out.println("mouse moved " + xWorld + ", " + yWorld + " sprite = " + sprite);
   }
   
   @Override
   public void mouseEntered(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
//	   System.out.println("mouse entered " + xWorld + ", " + yWorld);
   }

   @Override
   public void mouseExited(MouseEvent mouseEvent) 
   {
	   double xWorld = convertFromPanelPixelsToWorldX(mouseEvent.getX());
	   double yWorld = convertFromPanelPixelsToWorldY(mouseEvent.getY());
	   
//	   System.out.println("mouse exited " + xWorld + ", " + yWorld);
   }
}