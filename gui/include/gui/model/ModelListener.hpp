#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }

    virtual void setAQI (float aqi_val, float aqi25_calc, float aqi10_calc) {};

protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
