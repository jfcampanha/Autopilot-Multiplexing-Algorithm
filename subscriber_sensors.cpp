#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/time.hpp>
#include <vector>
#include <fort.hpp>

//flypt dsld files
#include <dsdl_source/Autopilot_1.hpp>
#include <dsdl_source/Autopilot_2.hpp>
#include <dsdl_source/Autopilot_3.hpp>

#include <dsdl_source/Auto1_control.hpp>
#include <dsdl_source/Auto2_control.hpp>
#include <dsdl_source/Auto3_control.hpp>

using dsdl_source::Autopilot_1;
using dsdl_source::Autopilot_2;
using dsdl_source::Autopilot_3;

using dsdl_source::Auto1_control;
using dsdl_source::Auto2_control;
using dsdl_source::Auto3_control;


extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

//uavcan::MonotonicTime timestam_old;

static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

//*********************************************************************************************************************************
//Function to define the size of the datatypes

template <typename AutopilotType>
void definingSystemSize(AutopilotType& auto_msg, const uint8_t number_modules, 
const uint8_t number_gnss, const uint8_t number_axes)
{
    //number of modules
    auto_msg.autopilot_index[0].acc.resize(number_modules);
    auto_msg.autopilot_index[0].airspeed.resize(number_modules);
    auto_msg.autopilot_index[0].baro.resize(number_modules);
    auto_msg.autopilot_index[0].gnss.resize(number_gnss); //gnss different
    auto_msg.autopilot_index[0].gyro.resize(number_modules);
    auto_msg.autopilot_index[0].mag.resize(number_modules);
    auto_msg.status_index[0].acc_status.resize(number_modules);
    auto_msg.status_index[0].airspeed_status.resize(number_modules);
    auto_msg.status_index[0].baro_status.resize(number_modules);
    auto_msg.status_index[0].gnss_status.resize(number_gnss);
    auto_msg.status_index[0].gyro_status.resize(number_modules);
    auto_msg.status_index[0].mag_status.resize(number_modules);

    //number of axes
    for (uint8_t j = 0; j < number_modules; j++)
    {
        auto_msg.autopilot_index[0].acc[j].temperature.resize(number_axes);
        auto_msg.autopilot_index[0].acc[j].error_count.resize(number_axes);
        auto_msg.autopilot_index[0].acc[j].rate.resize(number_axes);

        auto_msg.autopilot_index[0].gyro[j].temperature.resize(number_axes);
        auto_msg.autopilot_index[0].gyro[j].error_count.resize(number_axes);
        auto_msg.autopilot_index[0].gyro[j].rate.resize(number_axes);
    } 
}

//*********************************************************************************************************************************
//Auxiliar functions

float calculateMean(const float sum, const uint8_t number)
{
    float mean;
    mean = sum / number;

return mean;
}

float calculateVariance(const float sum, const uint8_t number)
{
    float variance;
    variance = sum / number;

return variance;
}

bool calculateDifference(const float variance, const float tolerance)
{
    bool trustness;

    if(tolerance>=variance)
    {
        trustness = 1; //trusted data
    }
    if(tolerance<variance)
    {
        trustness = 0; //untrusted data
    }
    else{};

return trustness;
}

//*********************************************************************************************************************************
//Function to check the tolerance and variance of all the data

template <typename Autopilot_x>
std::vector <bool> checkingTolerance(Autopilot_x& auto_msg, const uint8_t x, const uint8_t y, const uint8_t z, const uint8_t number_modules, 
const uint8_t number_gnss, const float tolerance)
{
    std::vector<float> sum(41, 0.0); //all sensor info type
    std::vector<float> mean(41, 0.0);
    std::vector<float> variance(41, 0.0);
    std::vector<bool> classification(41, 0.0);

    for(uint8_t j = 0; j < number_modules; j++)
    {
        sum[0] += auto_msg.autopilot_index[0].acc[j].accelerometer[x];
        sum[1] += auto_msg.autopilot_index[0].acc[j].accelerometer[y];
        sum[2] += auto_msg.autopilot_index[0].acc[j].accelerometer[z];
        sum[3] += auto_msg.autopilot_index[0].acc[j].temperature[x];
        sum[4] += auto_msg.autopilot_index[0].acc[j].temperature[y]; //no data, repeat
        sum[5] += auto_msg.autopilot_index[0].acc[j].temperature[z]; //no data, repeat
        sum[6] += auto_msg.autopilot_index[0].acc[j].error_count[x];
        sum[7] += auto_msg.autopilot_index[0].acc[j].error_count[y];
        sum[8] += auto_msg.autopilot_index[0].acc[j].error_count[z];
        sum[9] += auto_msg.autopilot_index[0].acc[j].rate[x];
        sum[10] += auto_msg.autopilot_index[0].acc[j].rate[y];
        sum[11] += auto_msg.autopilot_index[0].acc[j].rate[z]; //no data, repeat

        sum[12] += auto_msg.autopilot_index[0].airspeed[j].airspeed;
        sum[13] += auto_msg.autopilot_index[0].airspeed[j].temperature;

        sum[14] += auto_msg.autopilot_index[0].baro[j].altitude;
        sum[15] += auto_msg.autopilot_index[0].baro[j].pressure;
        sum[16] += auto_msg.autopilot_index[0].baro[j].temperature;

        sum[17] += auto_msg.autopilot_index[0].gyro[j].angular_velocity[x];
        sum[18] += auto_msg.autopilot_index[0].gyro[j].angular_velocity[y];
        sum[19] += auto_msg.autopilot_index[0].gyro[j].angular_velocity[z];
        sum[20] += auto_msg.autopilot_index[0].gyro[j].temperature[x];
        sum[21] += auto_msg.autopilot_index[0].gyro[j].temperature[y];//no data, repeat
        sum[22] += auto_msg.autopilot_index[0].gyro[j].temperature[z];//no data, repeat
        sum[23] += auto_msg.autopilot_index[0].gyro[j].error_count[x];
        sum[24] += auto_msg.autopilot_index[0].gyro[j].error_count[y];
        sum[25] += auto_msg.autopilot_index[0].gyro[j].error_count[z];
        sum[26] += auto_msg.autopilot_index[0].gyro[j].rate[x];
        sum[27] += auto_msg.autopilot_index[0].gyro[j].rate[y];
        sum[28] += auto_msg.autopilot_index[0].gyro[j].rate[z];//no data, repeat

        sum[29] += auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[x];
        sum[30] += auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[y];
        sum[31] += auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[z];

        if(j<number_gnss)
        {
            sum[32] += auto_msg.autopilot_index[0].gnss[j].status;
            sum[33] += auto_msg.autopilot_index[0].gnss[j].satellites_visible;
            sum[34] += auto_msg.autopilot_index[0].gnss[j].hdop;
            sum[35] += auto_msg.autopilot_index[0].gnss[j].vdop; //no data, repeat
            sum[36] += auto_msg.autopilot_index[0].gnss[j].latitude;
            sum[37] += auto_msg.autopilot_index[0].gnss[j].longitude;
            sum[38] += auto_msg.autopilot_index[0].gnss[j].altitude;
            sum[39] += auto_msg.autopilot_index[0].gnss[j].ground_speed;
            sum[40] += auto_msg.autopilot_index[0].gnss[j].ground_course;
        }else{}

        for(uint8_t i = 0; i < sum.size(); i++)
        {
            mean[i] = calculateMean(sum[i], number_modules);

            if(i>31)
            {
                mean[i] = calculateMean(sum[i], number_gnss);
            }
        }
    }

    std::fill_n(sum.begin(), 41, 0.0); //all the sum is clear
    //sum vector reused

    // for(uint8_t j = 0; j < number_modules; j++)
    // {
    //     sum[0] += pow(auto_msg.autopilot_index[0].acc[j].accelerometer[x] - mean[0], 2);
    //     sum[1] += pow(auto_msg.autopilot_index[0].acc[j].accelerometer[y] - mean[1], 2);
    //     sum[2] += pow(auto_msg.autopilot_index[0].acc[j].accelerometer[z] - mean[2], 2);
    //     sum[3] += pow(auto_msg.autopilot_index[0].acc[j].temperature[x] - mean[3], 2);
    //     sum[4] += pow(auto_msg.autopilot_index[0].acc[j].temperature[y] - mean[4], 2); //no data, repeat
    //     sum[5] += pow(auto_msg.autopilot_index[0].acc[j].temperature[z] - mean[5], 2); //no data, repeat
    //     sum[6] += pow(auto_msg.autopilot_index[0].acc[j].error_count[x] - mean[6], 2);
    //     sum[7] += pow(auto_msg.autopilot_index[0].acc[j].error_count[y] - mean[7], 2);
    //     sum[8] += pow(auto_msg.autopilot_index[0].acc[j].error_count[z] - mean[8], 2);
    //     sum[9] += pow(auto_msg.autopilot_index[0].acc[j].rate[x] - mean[9], 2);
    //     sum[10] += pow(auto_msg.autopilot_index[0].acc[j].rate[y] - mean[10], 2);
    //     sum[11] += pow(auto_msg.autopilot_index[0].acc[j].rate[z] - mean[11], 2); //no data, repeat

    //     sum[12] += pow(auto_msg.autopilot_index[0].airspeed[j].airspeed - mean[12], 2);
    //     sum[13] += pow(auto_msg.autopilot_index[0].airspeed[j].temperature - mean[13], 2);

    //     sum[14] += pow(auto_msg.autopilot_index[0].baro[j].altitude - mean[14], 2);
    //     sum[15] += pow(auto_msg.autopilot_index[0].baro[j].pressure - mean[15], 2);
    //     sum[16] += pow(auto_msg.autopilot_index[0].baro[j].temperature - mean[16], 2);

    //     sum[17] += pow(auto_msg.autopilot_index[0].gyro[j].angular_velocity[x] - mean[17], 2);
    //     sum[18] += pow(auto_msg.autopilot_index[0].gyro[j].angular_velocity[y] - mean[18], 2);
    //     sum[19] += pow(auto_msg.autopilot_index[0].gyro[j].angular_velocity[z] - mean[19], 2);
    //     sum[20] += pow(auto_msg.autopilot_index[0].gyro[j].temperature[x] - mean[20], 2);
    //     sum[21] += pow(auto_msg.autopilot_index[0].gyro[j].temperature[y] - mean[21], 2);//no data, repeat
    //     sum[22] += pow(auto_msg.autopilot_index[0].gyro[j].temperature[z] - mean[22], 2);//no data, repeat
    //     sum[23] += pow(auto_msg.autopilot_index[0].gyro[j].error_count[x] - mean[23], 2);
    //     sum[24] += pow(auto_msg.autopilot_index[0].gyro[j].error_count[y] - mean[24], 2);
    //     sum[25] += pow(auto_msg.autopilot_index[0].gyro[j].error_count[z] - mean[25], 2);
    //     sum[26] += pow(auto_msg.autopilot_index[0].gyro[j].rate[x] - mean[26], 2);
    //     sum[27] += pow(auto_msg.autopilot_index[0].gyro[j].rate[y] - mean[27], 2);
    //     sum[28] += pow(auto_msg.autopilot_index[0].gyro[j].rate[z] - mean[28], 2);//no data, repeat

    //     sum[29] += pow(auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[x] - mean[29], 2);
    //     sum[30] += pow(auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[y] - mean[30], 2);
    //     sum[31] += pow(auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[z] - mean[31], 2);

    //     if(j<number_gnss)
    //     {
    //         sum[32] += pow(auto_msg.autopilot_index[0].gnss[j].status -mean[32], 2);
    //         sum[33] += pow(auto_msg.autopilot_index[0].gnss[j].satellites_visible -mean[33], 2);
    //         sum[34] += pow(auto_msg.autopilot_index[0].gnss[j].hdop -mean[34], 2);
    //         sum[35] += pow(auto_msg.autopilot_index[0].gnss[j].vdop -mean[35], 2); //no data, repeat
    //         sum[36] += pow(auto_msg.autopilot_index[0].gnss[j].latitude -mean[36], 2);
    //         sum[37] += pow(auto_msg.autopilot_index[0].gnss[j].longitude -mean[37], 2);
    //         sum[38] += pow(auto_msg.autopilot_index[0].gnss[j].altitude -mean[38], 2);
    //         sum[39] += pow(auto_msg.autopilot_index[0].gnss[j].ground_speed -mean[39], 2);
    //         sum[40] += pow(auto_msg.autopilot_index[0].gnss[j].ground_course -mean[40], 2);
    //     }else{}
    // }


    for(uint8_t j = 0; j < number_modules; j++)
    {
        sum[0] += pow(auto_msg.autopilot_index[0].acc[j].accelerometer[x] - mean[0], 2);
        sum[1] += pow(auto_msg.autopilot_index[0].acc[j].accelerometer[y] - mean[1], 2);
        sum[2] += pow(auto_msg.autopilot_index[0].acc[j].accelerometer[z] - mean[2], 2);
        sum[3] += pow(auto_msg.autopilot_index[0].acc[j].temperature[x] - mean[3], 2);
        sum[4] += pow(auto_msg.autopilot_index[0].acc[j].temperature[y] - mean[4], 2); //no data, repeat
        sum[5] += pow(auto_msg.autopilot_index[0].acc[j].temperature[z] - mean[5], 2); //no data, repeat
        sum[6] += pow(auto_msg.autopilot_index[0].acc[j].error_count[x] - mean[6], 2);
        sum[7] += pow(auto_msg.autopilot_index[0].acc[j].error_count[y] - mean[7], 2);
        sum[8] += pow(auto_msg.autopilot_index[0].acc[j].error_count[z] - mean[8], 2);
        sum[9] += pow(auto_msg.autopilot_index[0].acc[j].rate[x] - mean[9], 2);
        sum[10] += pow(auto_msg.autopilot_index[0].acc[j].rate[y] - mean[10], 2);
        sum[11] += pow(auto_msg.autopilot_index[0].acc[j].rate[z] - mean[11], 2); //no data, repeat

        sum[12] += pow(auto_msg.autopilot_index[0].airspeed[j].airspeed - mean[12], 2);
        sum[13] += pow(auto_msg.autopilot_index[0].airspeed[j].temperature - mean[13], 2);

        sum[14] += pow(auto_msg.autopilot_index[0].baro[j].altitude - mean[14], 2);
        sum[15] += pow(auto_msg.autopilot_index[0].baro[j].pressure - mean[15], 2);
        sum[16] += pow(auto_msg.autopilot_index[0].baro[j].temperature - mean[16], 2);

        sum[17] += pow(auto_msg.autopilot_index[0].gyro[j].angular_velocity[x] - mean[17], 2);
        sum[18] += pow(auto_msg.autopilot_index[0].gyro[j].angular_velocity[y] - mean[18], 2);
        sum[19] += pow(auto_msg.autopilot_index[0].gyro[j].angular_velocity[z] - mean[19], 2);
        sum[20] += pow(auto_msg.autopilot_index[0].gyro[j].temperature[x] - mean[20], 2);
        sum[21] += pow(auto_msg.autopilot_index[0].gyro[j].temperature[y] - mean[21], 2);//no data, repeat
        sum[22] += pow(auto_msg.autopilot_index[0].gyro[j].temperature[z] - mean[22], 2);//no data, repeat
        sum[23] += pow(auto_msg.autopilot_index[0].gyro[j].error_count[x] - mean[23], 2);
        sum[24] += pow(auto_msg.autopilot_index[0].gyro[j].error_count[y] - mean[24], 2);
        sum[25] += pow(auto_msg.autopilot_index[0].gyro[j].error_count[z] - mean[25], 2);
        sum[26] += pow(auto_msg.autopilot_index[0].gyro[j].rate[x] - mean[26], 2);
        sum[27] += pow(auto_msg.autopilot_index[0].gyro[j].rate[y] - mean[27], 2);
        sum[28] += pow(auto_msg.autopilot_index[0].gyro[j].rate[z] - mean[28], 2);//no data, repeat
        
        sum[29] += pow(auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[x] - mean[29], 2);
        sum[30] += pow(auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[y] - mean[30], 2);
        sum[31] += pow(auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[z] - mean[31], 2);

        if(j<number_gnss)
        {
            sum[32] += pow(auto_msg.autopilot_index[0].gnss[j].status -mean[32], 2);
            sum[33] += pow(auto_msg.autopilot_index[0].gnss[j].satellites_visible -mean[33], 2);
            sum[34] += pow(auto_msg.autopilot_index[0].gnss[j].hdop -mean[34], 2);
            sum[35] += pow(auto_msg.autopilot_index[0].gnss[j].vdop -mean[35], 2); //no data, repeat
            sum[36] += pow(auto_msg.autopilot_index[0].gnss[j].latitude -mean[36], 2);
            sum[37] += pow(auto_msg.autopilot_index[0].gnss[j].longitude -mean[37], 2);
            sum[38] += pow(auto_msg.autopilot_index[0].gnss[j].altitude -mean[38], 2);
            sum[39] += pow(auto_msg.autopilot_index[0].gnss[j].ground_speed -mean[39], 2);
            sum[40] += pow(auto_msg.autopilot_index[0].gnss[j].ground_course -mean[40], 2);
        }else{}

    }

    for(uint8_t i = 0; i < sum.size(); i++)
    {
        variance[i] = calculateVariance(sum[i], number_modules);

        if(i>31)
        {
            variance[i] = calculateVariance(sum[i], number_gnss);
        }
    }

    for(uint8_t i = 0; i < sum.size(); i++)
    {
        classification[i] = calculateDifference(variance[i], tolerance);
        
    }

    return classification;
}

//*********************************************************************************************************************************
//Function to setting the status of the modules
template <typename Autox_control>
bool settingStatus(Autox_control& auto_control, const uint8_t count_sensors, const uint8_t number)
{
    bool checkcontrol = 0;
    if((count_sensors == number))
    {
        auto_control.WORKING++;
        checkcontrol = 1;
    }
    
    if(((count_sensors > 0) && (count_sensors < number)))
    {
        auto_control.ALERT++;
    }

    if((count_sensors == 0))
    {
        auto_control.FAILURE++;
    }

    return checkcontrol;
}

//*********************************************************************************************************************************
//Function to check the status of the modules
template <typename Autopilot_x, typename Autox_control>
float checkingStatus(Autopilot_x& auto_msg, Autox_control& auto_control, const uint8_t x,  const uint8_t y, const uint8_t z, 
const uint8_t number_modules, const uint8_t number_gnss, const float tolerance)
{   
    auto_control.WORKING = 0;
    auto_control.ALERT = 0;
    auto_control.FAILURE = 0;
    auto_control.UNTRUSTED = 0;

    auto_control.UNTRUSTED_ACC = 0;
    auto_control.UNTRUSTED_AIR = 0;
    auto_control.UNTRUSTED_BARO = 0;
    auto_control.UNTRUSTED_GYRO = 0;
    auto_control.UNTRUSTED_MAG = 0;
    auto_control.UNTRUSTED_GNSS = 0;

    auto_control.CHECKCONTROL_ACC = 0;
    auto_control.CHECKCONTROL_AIR = 0;
    auto_control.CHECKCONTROL_BARO = 0;
    auto_control.CHECKCONTROL_GYRO = 0;
    auto_control.CHECKCONTROL_MAG = 0;
    auto_control.CHECKCONTROL_GNSS = 0;

    uint16_t count_acc = 0;
    uint16_t count_air = 0;
    uint16_t count_baro = 0;
    uint16_t count_gyro = 0;
    uint16_t count_mag = 0;
    uint16_t count_gnss = 0;

    std::vector<bool> classification(41, 0.0);
    
    //std::cout << "--------" << std::endl;

    for (uint8_t j = 0; j < number_modules; j++)
    {
        if (auto_msg.autopilot_index[0].acc[j].health == 1)
        {
            count_acc++;
        }

        if (auto_msg.autopilot_index[0].airspeed[j].health == 1)
        {
            count_air++;
        }

        if (auto_msg.autopilot_index[0].baro[j].health == 1)
        {
            count_baro++;
        }

        if (auto_msg.autopilot_index[0].gyro[j].health == 1)
        {
            count_gyro++;
        }

        if (auto_msg.autopilot_index[0].mag[j].health == 1)
        {
            count_mag++;
        }

        if(j<number_gnss)
        {
            if (auto_msg.autopilot_index[0].gnss[j].health == 1)
            {
                count_gnss++;
            }else{}
        }else{}
    }

    auto_control.CHECKCONTROL_ACC = settingStatus<Autox_control>(auto_control, count_acc, number_modules);
    auto_control.CHECKCONTROL_AIR = settingStatus<Autox_control>(auto_control, count_air, number_modules);
    auto_control.CHECKCONTROL_BARO = settingStatus<Autox_control>(auto_control, count_baro, number_modules);
    auto_control.CHECKCONTROL_GYRO = settingStatus<Autox_control>(auto_control, count_gyro, number_modules);
    auto_control.CHECKCONTROL_MAG = settingStatus<Autox_control>(auto_control, count_mag, number_modules);
    auto_control.CHECKCONTROL_GNSS = settingStatus<Autox_control>(auto_control, count_gnss, number_gnss);

    //health = 1 -> checks the variance and tolerance
    //classification being 0 means that the variance as exceed the user defined tolerance
    classification = checkingTolerance<Autopilot_x>(auto_msg, x, y, z, number_modules, number_gnss, tolerance);


    //just check if the the healh of one module is set to 1
    if(auto_control.CHECKCONTROL_ACC == 1)
    {
        for(uint8_t k = 0; k<11; k++)//[0,11]
        {
            if(classification[k] == 0)
            {
                auto_control.UNTRUSTED_ACC = 1;
            }
        }
        if(auto_control.UNTRUSTED_ACC == 1)
        {
            auto_control.UNTRUSTED ++;
        }
    }

    if(auto_control.CHECKCONTROL_AIR == 1)
    {
        for(uint8_t k = 12; k<13; k++)// [12,13]
        {
            if(classification[k] == 0)
            {
                auto_control.UNTRUSTED_AIR = 1;
            } 
        }
        if(auto_control.UNTRUSTED_AIR == 1)
        {
            auto_control.UNTRUSTED ++;
        }
    }

    if(auto_control.CHECKCONTROL_BARO == 1)
    {
        for(uint8_t k = 14; k<16; k++)//[14,16]
        {
            if(classification[k] == 0)
            {
                auto_control.UNTRUSTED_BARO = 1;
            }
        }
        if(auto_control.UNTRUSTED_BARO == 1)
        {
            auto_control.UNTRUSTED ++;
        }
    }


    if(auto_control.CHECKCONTROL_GYRO == 1)
    {
        for(uint8_t k = 17; k<28; k++)//[17,28]
        {
            if(classification[k] == 0)
            {
                auto_control.UNTRUSTED_GYRO = 1;
            }
        }
        if(auto_control.UNTRUSTED_GYRO == 1)
        {
            auto_control.UNTRUSTED ++;
        }
    }


    if(auto_control.CHECKCONTROL_MAG == 1)
    {
        for(uint8_t k = 29; k<31; k++)//[29, 31]
        {
            if(classification[k] == 0)
            {
                auto_control.UNTRUSTED_MAG = 1;
            }
        }
        if(auto_control.UNTRUSTED_MAG == 1)
        {
            auto_control.UNTRUSTED ++;
        }
    }


    if(auto_control.CHECKCONTROL_GNSS == 1)
    {
        for(uint8_t k = 32; k<40; k++)//[32,40]
        {
            if(classification[k] == 0)
            {
                auto_control.UNTRUSTED_GNSS = 1;
            }
        }
        if(auto_control.UNTRUSTED_GNSS == 1)
        {
            auto_control.UNTRUSTED ++;
        }
    }

    //calculates the mean value of the autopilot based on the sensor information
    float mean_auto = (float)auto_control.WORKING*(1.0f/6.0f) + (float)auto_control.ALERT*(1.0f/12.0f) + 
                      (float)auto_control.FAILURE*0 -(float)auto_control.UNTRUSTED*(1.0f/6.0f);
    
    //std::cout << "Mean " << mean_auto << std::endl;
    return mean_auto;
}

//*********************************************************************************************************************************
//chooses the best autopilot

void autopilotChoice(uint16_t&auto_control, float mean_auto1, float mean_auto2, float mean_auto3)
{
    if(mean_auto1 >= mean_auto2 && mean_auto1 >= mean_auto3)
    {
        auto_control = 1;
    }
    else if(mean_auto2 >= mean_auto1 && mean_auto2 >= mean_auto3)
    {
        auto_control = 2;
    }
    else
    {
        auto_control = 3;
    }
}
//*********************************************************************************************************************************
//Main function

int main(int argc, const char** argv)
{

    uint8_t number_modules = 3;
    uint8_t number_gnss = 2;
    uint8_t x=0, y=1, z=2; //3 dimmensional axes

    dsdl_source::Auto1_control auto1_control;
    dsdl_source::Auto2_control auto2_control;
    dsdl_source::Auto3_control auto3_control;

    float mean_auto1;
    float mean_auto2;
    float mean_auto3;

    float tolerance = 0;

    uint n = 1;
    uint16_t auto_control;

    fort::utf8_table table;
    table.set_border_style(FT_DOUBLE2_STYLE);

    uint8_t init = 0;
    uavcan::MonotonicTime timestamp_old;
    uavcan::MonotonicDuration elapsed_time;
    
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <node1-id>" << std::endl;
        return 1;
    }
    
    const int self_node1_id = std::stoi(argv[1]);
    //const int self_node2_id = std::stoi(argv[2]);

    auto& node1 = getNode();
    node1.setNodeID(self_node1_id);
    node1.setName("autopilot1_node");

    do {
        std::cout << "Tolerance for the module variance [2, 10]%: ";
        std::cin >> tolerance;
    } while (tolerance < 2 || tolerance > 10);

    std::cout << "Valid tolerance entered: " << tolerance << std::endl;
    
//*********************************************************************************************************************************  
//Registing the Data Type IDs.
//SensorStatus and Sensors Data already registed: 20986, 20987

    auto regist_result1 = uavcan::GlobalDataTypeRegistry::instance().registerDataType<Autopilot_1>(333);
    if (regist_result1 != uavcan::GlobalDataTypeRegistry::RegistrationResultOk)
    {
        throw std::runtime_error("Failed to register the data type: " + std::to_string(regist_result1));
    }

    auto regist_result2 = uavcan::GlobalDataTypeRegistry::instance().registerDataType<Autopilot_2>(334);
    if (regist_result2 != uavcan::GlobalDataTypeRegistry::RegistrationResultOk)
    {
        throw std::runtime_error("Failed to register the data type: " + std::to_string(regist_result2));
    }

    auto regist_result3 = uavcan::GlobalDataTypeRegistry::instance().registerDataType<Autopilot_3>(335);
    if (regist_result3 != uavcan::GlobalDataTypeRegistry::RegistrationResultOk)
    {
        throw std::runtime_error("Failed to register the data type: " + std::to_string(regist_result3));
    }

//*********************************************************************************************************************************  
//Starting Node 1

    const int node1_start_res = node1.start();
    if (node1_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node 1; error: " + std::to_string(node1_start_res));
    }
    
//********************************************************************************************************************************* 
//Starting the Subscribers

    uavcan::Subscriber<dsdl_source::Autopilot_1> auto1_sub(node1);

    const int auto1_sub_start_res = auto1_sub.start([&](const uavcan::ReceivedDataStructure<dsdl_source::Autopilot_1>& auto_msg1)
        { 
            mean_auto1 = checkingStatus(auto_msg1, auto1_control, x, y, z, number_modules, number_gnss, tolerance);
        });

    if (auto1_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the autopilot 1 subscriber; error: " + std::to_string(auto1_sub_start_res));
    }


    uavcan::Subscriber<dsdl_source::Autopilot_2> auto2_sub(node1);

    const int auto2_sub_start_res = auto2_sub.start([&](const uavcan::ReceivedDataStructure<dsdl_source::Autopilot_2>& auto_msg2)
        { 
            mean_auto2 = checkingStatus(auto_msg2, auto2_control, x, y, z, number_modules, number_gnss, tolerance);
        });

    if (auto2_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the autopilot 2 subscriber; error: " + std::to_string(auto2_sub_start_res));
    }

    //printing the information in the last reciever
    uavcan::Subscriber<dsdl_source::Autopilot_3> auto3_sub(node1);

    const int auto3_sub_start_res = auto3_sub.start([&](const uavcan::ReceivedDataStructure<dsdl_source::Autopilot_3>& auto_msg3)
        {
            mean_auto3 = checkingStatus(auto_msg3, auto3_control, x, y, z, number_modules, number_gnss, tolerance);
            uavcan::MonotonicTime timestamp = auto_msg3.getMonotonicTimestamp();
            if (init==0)
            {
                //creating table header
                table << fort::header
                << "" << "Control System" << "" << "" << "Autopilot 1" << "" << ""  << "Autopilot 2" << "" << ""  << "Autopilot 3" <<fort::endr;
                table[0][0].set_cell_empty_str_height(1);
                table[0][1].set_cell_span(3);
                table[0][4].set_cell_span(3);
                table[0][7].set_cell_span(3);
                table[0][10].set_cell_span(3);

                table
                << "Time (s)" << "Auto 1" << "Auto 2" << "Auto 3" 
                << "Failure" << "Alert" << "Untrusted" 
                << "Failure" << "Alert" << "Untrusted"
                << "Failure" << "Alert" << "Untrusted" << fort::endr;

                //changing aspect
                table.column(0).set_cell_text_align(fort::text_align::center);
                table.column(1).set_cell_text_align(fort::text_align::center);
                table.column(2).set_cell_text_align(fort::text_align::center);
                table.column(3).set_cell_text_align(fort::text_align::center);
                table.column(4).set_cell_text_align(fort::text_align::center);
                table.column(5).set_cell_text_align(fort::text_align::center);
                table.column(6).set_cell_text_align(fort::text_align::center);
                table.column(7).set_cell_text_align(fort::text_align::center);
                table.column(8).set_cell_text_align(fort::text_align::center);
                table.column(9).set_cell_text_align(fort::text_align::center);
                table.column(10).set_cell_text_align(fort::text_align::center);
                table.column(11).set_cell_text_align(fort::text_align::center);
                table.column(12).set_cell_text_align(fort::text_align::center);
                table.column(13).set_cell_text_align(fort::text_align::center);
                table[0][1].set_cell_content_fg_color(fort::color::yellow);
                table[0][4].set_cell_content_fg_color(fort::color::blue);
                table[0][7].set_cell_content_fg_color(fort::color::green);
                table[0][10].set_cell_content_fg_color(fort::color::magenta);
                table[1][1].set_cell_content_fg_color(fort::color::blue);
                table[1][2].set_cell_content_fg_color(fort::color::green);
                table[1][3].set_cell_content_fg_color(fort::color::magenta);

            }else{ 
                elapsed_time += (timestamp - timestamp_old);//time passed beetween iterations

                //chosing the best autopilot
                autopilotChoice(auto_control, mean_auto1, mean_auto2, mean_auto3);

                if (auto_control == 1)
                {
                    table << elapsed_time << "✔" << "-" << "-" 
                    << auto1_control.FAILURE << auto1_control.ALERT  << auto1_control.UNTRUSTED
                    << auto2_control.FAILURE << auto2_control.ALERT  << auto2_control.UNTRUSTED
                    << auto3_control.FAILURE << auto3_control.ALERT  << auto3_control.UNTRUSTED   
                    << fort::endr;
                    table[n][auto_control].set_cell_content_fg_color(fort::color::green);

                }
                if (auto_control == 2)
                {
                    table << elapsed_time << "-" << "✔" << "-"
                    << auto1_control.FAILURE << auto1_control.ALERT  << auto1_control.UNTRUSTED
                    << auto2_control.FAILURE << auto2_control.ALERT  << auto2_control.UNTRUSTED
                    << auto3_control.FAILURE << auto3_control.ALERT  << auto3_control.UNTRUSTED 
                    << fort::endr;
                    table[n][auto_control].set_cell_content_fg_color(fort::color::green);

                }
                if (auto_control == 3)
                {
                    table << elapsed_time << "-" << "-" << "✔" 
                    << auto1_control.FAILURE << auto1_control.ALERT  << auto1_control.UNTRUSTED
                    << auto2_control.FAILURE << auto2_control.ALERT  << auto2_control.UNTRUSTED
                    << auto3_control.FAILURE << auto3_control.ALERT  << auto3_control.UNTRUSTED 
                    << fort::endr;
                    table[n][auto_control].set_cell_content_fg_color(fort::color::green);
                }
            }
            
            timestamp_old = timestamp;
            init = 1;
            int8_t result = system("clear");
            if (result == -1) 
            {}
            std::cout << table.to_string();//updates the terminal
            n++;
        });

    if (auto3_sub_start_res < 0)
    {
        throw std::runtime_error("Failed to start the autopilot 3 subscriber; error: " + std::to_string(auto3_sub_start_res));
    }
     
    node1.setModeOperational();

    while (true)
    {
        const int res = node1.spin(uavcan::MonotonicDuration::getInfinite());
        if (res < 0)
        {
            std::cerr << "Transient failure: " << res << std::endl;
        }
    }
}
