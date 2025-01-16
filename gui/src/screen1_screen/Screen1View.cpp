#include <gui/screen1_screen/Screen1View.hpp>

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

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

void Screen1View::setAQI (float aqi_val, float aqi25_calc, float aqi10_calc)
{

	int aqi_arc_val  = map(int(aqi_val)*10 + (int)(aqi_val * 10) % 10, 0, 100, 220, 500);
	int pm25_arc_val = map(int(aqi25_calc)*10 + (int)(aqi25_calc * 10) % 10, 0, 100, 220, 500);
	int pm10_arc_val = map(int(aqi10_calc)*10 + (int)(aqi10_calc * 10) % 10, 0, 100, 220, 500);



    //aqi_indexArcPainter.setColor(touchgfx::Color::getColorFromRGB(119, 242, 255));
    //aqi_indexArc.setPainter(aqi_indexArc_totPainter);
    //aqi_indexArc.setAlpha(18);
	aqi_indexArc.invalidate();


	aqi_indexArcPainter.setColor(touchgfx::Color::getColorFromRGB(119, 242, 255));
    aqi_indexArc.setPainter(aqi_indexArcPainter);
	aqi_indexArc.setArc(220, aqi_arc_val);
	aqi_indexArc.invalidate();


    //pm25ArcPainter.setColor(touchgfx::Color::getColorFromRGB(255, 134, 88));
    //pm25Arc.setPainter(pm25Arc_1Painter);
    //pm25Arc.setAlpha(51);
    pm25Arc.invalidate();

    pm25ArcPainter.setColor(touchgfx::Color::getColorFromRGB(255, 134, 88));
    pm25Arc.setPainter(pm25Arc_1Painter);
	pm25Arc.setArc(220, pm25_arc_val);
	pm25Arc.invalidate();


    //co2ArcPainter.setColor(touchgfx::Color::getColorFromRGB(204, 89, 72));
    //co2Arc.setPainter(co2Arc_1Painter);
    //co2Arc.setAlpha(36);
    co2Arc.invalidate();

    co2ArcPainter.setColor(touchgfx::Color::getColorFromRGB(204, 89, 72));
    co2Arc.setPainter(co2Arc_1Painter);
	co2Arc.setArc(220, pm10_arc_val);
	co2Arc.invalidate();


	Unicode::snprintf(aqiTextBuffer, AQITEXT_SIZE, "%i.%i\r\n", (int)aqi_val, (int)(aqi_val * 10) % 10);
	aqiText.invalidate();
}
