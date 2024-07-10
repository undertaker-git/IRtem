#include "target.h"

void Target::SetTargetRect(cv::Rect rect){
    this->rect = rect;
}

cv::Rect Target::GetTargetRect(){
    return this->rect;
}

void Target::SetTargetTemperature(float temperature){
    this->Temperature = temperature;
}


void Target::UpdatePreviousTemperature(){
    this->temperature_Previous = this->Temperature;
}

float Target::GetTargetPreviousTemperature(){
    return this->temperature_Previous;
}


float Target::GetTargetTemperature(){
    return this->Temperature;
}

float Target::GetMajorityTemperature(float temperature) {
    float ret = 0.0;
    if (this->locate != 5)
    {
        ret=temperature;
        return ret;
    }
    qSort(this->TemperatureArray.begin(), this->TemperatureArray.end());
    int cnt = 1, sum = 1;
    for (int i = 1; i < this->TemperatureArray.size(); i++)
    {
        std::cout << "this->TemperatureArray[i] " << this->TemperatureArray[i] << std::endl;
        if (abs(this->TemperatureArray[i] - this->TemperatureArray[i - 1]) < 1e-4)
        {
            cnt++;
        }
        else
        {
            cnt = 1;
        }
        if (cnt >= sum)
        {
            sum = cnt;
            ret = this->TemperatureArray[i];
        }
    }
    return ret;
}


bool Target::initTrack( InputArray image, const Rect2d& boundingBox ){
    tracker = TrackerCSRT::create();
    return tracker->init(image, boundingBox);
}

void Target::addLocate(){
    locate++;
}

int Target::getLocate(){
    return locate;
}
