#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{
	lightState = false;
}

void Screen1Presenter::deactivate()
{

}

void Screen1Presenter::toggleLight()
{
    if (lightState == false)
    {
      lightState = true;
      view.switchBulbOn();
    }
    else
    {
      lightState = false;
      view.switchBulbOff();
    }
}

void Screen1Presenter::changeColor()
{
	view.showchange();
}
