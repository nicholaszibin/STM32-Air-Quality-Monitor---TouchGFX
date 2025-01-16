#ifndef MODEL_HPP
#define MODEL_HPP

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdint>
#include <stdlib.h>
#include <limits.h>
#include <float.h>

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();


protected:
    ModelListener* modelListener;
    float aqi_val;
    float aqi25_calc;
	float aqi10_calc;

};

#endif // MODEL_HPP
