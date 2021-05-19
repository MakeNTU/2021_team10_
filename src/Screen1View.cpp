#include <gui/screen1_screen/Screen1View.hpp>
#include "BitmapDatabase.hpp"
#include <touchgfx/Color.hpp>


Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::switchBulbOn()
{
  bulb.setBitmap(Bitmap(BITMAP_BULB_ON_ID));
  bulb.invalidate();
}

void Screen1View::switchBulbOff()
{
  bulb.setBitmap(Bitmap(BITMAP_BULB_OFF_ID));
  bulb.invalidate();
}

void Screen1View::showchange()
{
	static int i = 0;
	if(i % 5 == 0)
		Lp1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));
	else if(i % 5 == 1)
		Lp1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 0, 0));
	else if(i % 5 == 2)
		Lp1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 255, 0));
	else if(i % 5 == 3)
		Lp1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 255));
	else
		Lp1Painter.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));

	Lp1.setPainter(Lp1Painter);
	Lp1.setValue(i);

	Lp1.invalidate();


	if(i > 99) i-=99;
	else i+=1;
}
