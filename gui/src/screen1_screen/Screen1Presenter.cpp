#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}

void Screen1Presenter::setAQI (float aqi_val, float aqi25_calc, float aqi10_calc)
{
	view.setAQI (aqi_val, aqi25_calc, aqi10_calc);
}
