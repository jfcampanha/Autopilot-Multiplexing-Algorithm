#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <ctime>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <uavcan/uavcan.hpp>

//flypt dsld files
#include <dsdl_source/Autopilot_1.hpp>
#include <dsdl_source/Autopilot_2.hpp>
#include <dsdl_source/Autopilot_3.hpp>

using dsdl_source::Autopilot_1;
using dsdl_source::Autopilot_2;
using dsdl_source::Autopilot_3;

extern uavcan::ICanDriver& getCanDriver();
extern uavcan::ISystemClock& getSystemClock();

constexpr unsigned NodeMemoryPoolSize = 16384;
typedef uavcan::Node<NodeMemoryPoolSize> Node;

//Setting the clock and driver for the node
static Node& getNode()
{
    static Node node(getCanDriver(), getSystemClock());
    return node;
}

//********************************************************************************************************************************* 
//Functions for data loading from .csv files

void load_ARSP(std::vector<float>& airspeed, std::vector<float>& temperature, std::vector<bool>& health)
{
    std::ifstream data("../csv_data/ARSP.csv");
    if (!data.is_open())
    {
        std::exit(EXIT_FAILURE);
    }
    std::string str;
    std::getline(data, str); // skip the first line for the variables name

    while (std::getline(data, str))
    {
        std::istringstream iss(str);
        std::string col1, col2, col3;
        std::getline(iss, col1, ',');
        std::getline(iss, col2, ',');
        std::getline(iss, col3, ',');

        airspeed.push_back(std::stof(col1));
        temperature.push_back(std::stof(col2));
        health.push_back(std::stoi(col3));

    }
    data.close();

    if (data.eof())
    {
        std::cout << "AIR.csv loaded. \n" << std::endl;
    }
    else
    {
        std::cout << "Failed to load ARSP. \n" << std::endl;
    }
}

void load_BAR(std::vector<float>& altitude, std::vector<float>& pressure, std::vector<float>& temperature, std::vector<bool>& health)
{
    std::ifstream data("../csv_data/BAR.csv");
    if (!data.is_open())
    {
        std::exit(EXIT_FAILURE);
    }
    std::string str;
    std::getline(data, str); // skip the first line for the variables name

    while (std::getline(data, str))
    {
        std::istringstream iss(str);
        std::string col1, col2, col3, col4;
        std::getline(iss, col1, ',');
        std::getline(iss, col2, ',');
        std::getline(iss, col3, ',');
        std::getline(iss, col4, ',');

        altitude.push_back(std::stof(col1));
        pressure.push_back(std::stof(col2));
        temperature.push_back(std::stof(col3));
        health.push_back(std::stoi(col4));

    }
    data.close();

    if (data.eof())
    {
        std::cout << "BAR.csv loaded. \n" << std::endl;
    }
    else
    {
        std::cout << "Failed to load BAR.csv \n" << std::endl;
    }
}

void load_GPS(std::vector<int>& status, std::vector<int>& n_satellites, std::vector<float>& hdop, std::vector<float>& latitude, 
std::vector<float>& longitude, std::vector<float>& altitude, std::vector<float>& speed, std::vector<float>& gcrs, std::vector<bool>& health)
{
    std::ifstream data("../csv_data/GPS.csv");
    if (!data.is_open())
    {
        std::exit(EXIT_FAILURE);
    }
    std::string str;
    std::getline(data, str); // skip the first line for the variables name

    while (std::getline(data, str))
    {
        std::istringstream iss(str);
        std::string col1, col2, col3, col4, col5, col6, col7, col8, col9;
        std::getline(iss, col1, ',');
        std::getline(iss, col2, ',');
        std::getline(iss, col3, ',');
        std::getline(iss, col4, ',');
        std::getline(iss, col5, ',');
        std::getline(iss, col6, ',');
        std::getline(iss, col7, ',');
        std::getline(iss, col8, ',');
        std::getline(iss, col9, ',');
        status.push_back(std::stoi(col1));
        n_satellites.push_back(std::stof(col2));
        hdop.push_back(std::stof(col3));
        latitude.push_back(std::stof(col4));
        longitude.push_back(std::stof(col5));
        altitude.push_back(std::stof(col6));
        speed.push_back(std::stof(col7));
        gcrs.push_back(std::stof(col8));
        health.push_back(std::stoi(col9));

    }
    data.close();

    if (data.eof())
    {
        std::cout << "GPS.csv loaded. \n" << std::endl;
    }
    else
    {
        std::cout << "Failed to load GPS.csv \n" << std::endl;
    }
}

void load_IMU(std::vector<float>& gyrx, std::vector<float>& gyry, std::vector<float>& gyrz, std::vector<float>& accx, 
std::vector<float>& accy, std::vector<float>& accz, std::vector<float>& temperature_gyro, std::vector<float>& temperature_acc, std::vector<int>& rate1, std::vector<int>& rate2, 
std::vector<bool>& health_gyro, std::vector<bool>& health_acc)
{
    std::ifstream data("../csv_data/IMU.csv");
    if (!data.is_open())
    {
        std::exit(EXIT_FAILURE);
    }
    std::string str;
    std::getline(data, str); // skip the first line for the variables name

    while (std::getline(data, str))
    {
        std::istringstream iss(str);
        std::string col1, col2, col3, col4, col5, col6, col7, col8, col9, col10, col11, col12;
        std::getline(iss, col1, ',');
        std::getline(iss, col2, ',');
        std::getline(iss, col3, ',');
        std::getline(iss, col4, ',');
        std::getline(iss, col5, ',');
        std::getline(iss, col6, ',');
        std::getline(iss, col7, ',');
        std::getline(iss, col8, ',');
        std::getline(iss, col9, ',');
        std::getline(iss, col10, ',');
        std::getline(iss, col11, ',');
        std::getline(iss, col12, ',');
        gyrx.push_back(std::stof(col1));
        gyry.push_back(std::stof(col2));
        gyrz.push_back(std::stof(col3));
        accx.push_back(std::stof(col4));
        accy.push_back(std::stof(col5));
        accz.push_back(std::stof(col6));
        temperature_gyro.push_back(std::stof(col7));
        temperature_acc.push_back(std::stof(col8));
        rate1.push_back(std::stoi(col9));
        rate2.push_back(std::stoi(col10));
        health_gyro.push_back(std::stoi(col11));
        health_acc.push_back(std::stoi(col12));

    }
    data.close();

    if (data.eof())
    {
        std::cout << "IMU.csv loaded. \n" << std::endl;
    }
    else
    {
        std::cout << "Failed to load IMU.csv \n" << std::endl;
    }
}

void load_MAG(std::vector<int>& magx, std::vector<int>& magy, std::vector<int>& magz, std::vector<bool>& health)
{
    std::ifstream data("../csv_data/MAG.csv");
    if (!data.is_open())
    {
        std::exit(EXIT_FAILURE);
    }
    std::string str;
    std::getline(data, str); // skip the first line for the variables name

    while (std::getline(data, str))
    {
        std::istringstream iss(str);
        std::string col1, col2, col3, col4;
        std::getline(iss, col1, ',');
        std::getline(iss, col2, ',');
        std::getline(iss, col3, ',');
        std::getline(iss, col4, ',');
        magx.push_back(std::stoi(col1));
        magy.push_back(std::stoi(col2));
        magz.push_back(std::stoi(col3));
        health.push_back(std::stoi(col4));

    }
    data.close();

    if (data.eof())
    {
        std::cout << "MAG.csv loaded. \n" << std::endl;
    }
    else
    {
        std::cout << "Failed to load MAG.csv \n" << std::endl;
    }
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
//Function to assign values of the sensors database
template <typename AutopilotType>
void assignValues(AutopilotType& auto_msg, uint r, uint8_t x,  uint8_t y, uint8_t z, uint8_t number_modules, uint8_t number_gnss,

                const std::vector<float> air_airspeed, const std::vector<float> air_temperature, 
                const std::vector<float> bar_altitude, const std::vector<float> bar_pressure, 
                const std::vector<float> bar_temperature, const std::vector<float> gnss_hdop, 
                const std::vector<float> gnss_latitude, const std::vector<float> gnss_longitude, 
                const std::vector<float> gnss_altitude, const std::vector<float> gnss_speed, 
                const std::vector<float> gnss_gcrs, const std::vector<float> gyro_gyrox, 
                const std::vector<float> gyro_gyroy, const std::vector<float> gyro_gyroz, 
                const std::vector<float> acc_accx, const std::vector<float> acc_accy, 
                const std::vector<float> acc_accz, const std::vector<float> gyro_temp, 
                const std::vector<float> acc_temp,

                const std::vector<int> gnss_status, const std::vector<int> gnss_n_satellites, 
                const std::vector<int> acc_gyro_rate1, const std::vector<int> acc_gyro_rate2, 
                const std::vector<int> mag_magx, const std::vector<int> mag_magy, const std::vector<int> mag_magz,

                const std::vector<bool> air_health, const std::vector<bool> bar_health, 
                const std::vector<bool> gnss_health, const std::vector<bool> gyro_health, 
                const std::vector<bool> acc_health, const std::vector<bool> mag_health)
{
    uint origin_r = r;

    for(uint8_t j = 0; j < number_modules; j++)
    {
        //this ensures diverity of the values for sensor each module
        //values are aproximadated but not equal, being more realistic
        if(j==0){r=origin_r-(rand()% 3 + 1);}
        if(j==1){r=origin_r;}
        if(j==2){r=origin_r+(rand()% 3 + 1);}

        auto_msg.autopilot_index[0].acc[j].accelerometer[x] = acc_accx[r];
        auto_msg.autopilot_index[0].acc[j].accelerometer[y] = acc_accy[r];
        auto_msg.autopilot_index[0].acc[j].accelerometer[z] = acc_accz[r];
        auto_msg.autopilot_index[0].acc[j].temperature[x] = acc_temp[r];
        auto_msg.autopilot_index[0].acc[j].temperature[y] = acc_temp[r]; //no data, repeat
        auto_msg.autopilot_index[0].acc[j].temperature[z] = acc_temp[r]; //no data, repeat
        auto_msg.autopilot_index[0].acc[j].error_count[x] = r;
        auto_msg.autopilot_index[0].acc[j].error_count[y] = r;
        auto_msg.autopilot_index[0].acc[j].error_count[z] = r;
        auto_msg.autopilot_index[0].acc[j].rate[x] = acc_gyro_rate1[r];
        auto_msg.autopilot_index[0].acc[j].rate[y] = acc_gyro_rate2[r];
        auto_msg.autopilot_index[0].acc[j].rate[z] = acc_gyro_rate2[r]; //no data, repeat
        auto_msg.autopilot_index[0].acc[j].health = acc_health[r];  

        auto_msg.autopilot_index[0].airspeed[j].airspeed = air_airspeed[r];
        auto_msg.autopilot_index[0].airspeed[j].temperature = air_temperature[r];
        auto_msg.autopilot_index[0].airspeed[j].health = air_health[r];

        auto_msg.autopilot_index[0].baro[j].altitude = bar_altitude[r];
        auto_msg.autopilot_index[0].baro[j].pressure = bar_pressure[r];
        auto_msg.autopilot_index[0].baro[j].temperature = bar_temperature[r];
        auto_msg.autopilot_index[0].baro[j].health = bar_health[r];

        auto_msg.autopilot_index[0].gyro[j].angular_velocity[x] = gyro_gyrox[r];
        auto_msg.autopilot_index[0].gyro[j].angular_velocity[y] = gyro_gyroy[r];
        auto_msg.autopilot_index[0].gyro[j].angular_velocity[z] = gyro_gyroz[r];
        auto_msg.autopilot_index[0].gyro[j].temperature[x] = gyro_temp[r];
        auto_msg.autopilot_index[0].gyro[j].temperature[y] = gyro_temp[r];//no data, repeat
        auto_msg.autopilot_index[0].gyro[j].temperature[z] = gyro_temp[r];//no data, repeat
        auto_msg.autopilot_index[0].gyro[j].error_count[x] = r;
        auto_msg.autopilot_index[0].gyro[j].error_count[y] = r;
        auto_msg.autopilot_index[0].gyro[j].error_count[z] = r;
        auto_msg.autopilot_index[0].gyro[j].rate[x] = acc_gyro_rate1[r];
        auto_msg.autopilot_index[0].gyro[j].rate[y] = acc_gyro_rate2[r];
        auto_msg.autopilot_index[0].gyro[j].rate[z] = acc_gyro_rate2[r];//no data, repeat
        auto_msg.autopilot_index[0].gyro[j].health = gyro_health[r];

        auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[x] = mag_magx[r];
        auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[y] = mag_magy[r];
        auto_msg.autopilot_index[0].mag[j].magnetic_field_ga[z] = mag_magz[r];
        auto_msg.autopilot_index[0].mag[j].health = mag_health[r];
        
        //GNSS just have a max of 2 modules
        if(j<number_gnss)
        {
            auto_msg.autopilot_index[0].gnss[j].status = gnss_status[r];
            auto_msg.autopilot_index[0].gnss[j].satellites_visible = gnss_n_satellites[r];
            auto_msg.autopilot_index[0].gnss[j].hdop = gnss_hdop[r];
            auto_msg.autopilot_index[0].gnss[j].vdop = gnss_hdop[r]; //no data, repeat
            auto_msg.autopilot_index[0].gnss[j].latitude = gnss_latitude[r];
            auto_msg.autopilot_index[0].gnss[j].longitude = gnss_longitude[r];
            auto_msg.autopilot_index[0].gnss[j].altitude= gnss_altitude[r];
            auto_msg.autopilot_index[0].gnss[j].ground_speed = gnss_speed[r];
            auto_msg.autopilot_index[0].gnss[j].ground_course = gnss_gcrs[r];
            auto_msg.autopilot_index[0].gnss[j].health = gnss_health[r];
            
            //LOADING GNSS STATUS
            auto_msg.status_index[0].gnss_status[j] = auto_msg.autopilot_index[0].gnss[j].health;
        }else{}

        //LOADING STATUS
        auto_msg.status_index[0].acc_status[j] = auto_msg.autopilot_index[0].acc[j].health;
        auto_msg.status_index[0].airspeed_status[j] = auto_msg.autopilot_index[0].airspeed[j].health;
        auto_msg.status_index[0].baro_status[j] = auto_msg.autopilot_index[0].baro[j].health;
        auto_msg.status_index[0].gyro_status[j] = auto_msg.autopilot_index[0].gyro[j].health;
        auto_msg.status_index[0].mag_status[j] = auto_msg.autopilot_index[0].mag[j].health;

    } 
}

//*********************************************************************************************************************************
//Main Function 

int main(int argc, const char** argv)
{
    //reading the nodes from the terminal
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <node1-id> <node2-id> <node3-id>" << std::endl;
        return 1;
    }

    const int self_node1_id = std::stoi(argv[1]);
    const int self_node2_id = std::stoi(argv[2]);
    const int self_node3_id = std::stoi(argv[3]);

    auto& node1 = getNode();
    node1.setNodeID(self_node1_id);
    node1.setName("autopilot1_node");

    auto& node2 = getNode();
    node2.setNodeID(self_node2_id);
    node2.setName("autopilot2_node");

    auto& node3 = getNode();
    node3.setNodeID(self_node3_id);
    node3.setName("autopilot3_node");

    //*********************************************************************************************************************************
    //Registing the Data Type IDs for the Autopilots

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
    //Making sure that nodes are started
    const int node1_start_res = node1.start();
    if (node1_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node 1; error: " + std::to_string(node1_start_res));
    }

    const int node2_start_res = node2.start();
    if (node2_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node 2; error: " + std::to_string(node2_start_res));
    }

    const int node3_start_res = node3.start();
    if (node3_start_res < 0)
    {
        throw std::runtime_error("Failed to start the node 3; error: " + std::to_string(node3_start_res));
    }
    //*********************************************************************************************************************************
    //Starting the Publishers
    //NODE 1 = AUTOPILOT 1
    uavcan::Publisher<dsdl_source::Autopilot_1> auto1_pub(node1);
    const int auto1_pub_init_res = auto1_pub.init();
    if (auto1_pub_init_res < 0)
    {
        throw std::runtime_error("Failed to start the publisher Autopilot; error: " + std::to_string(auto1_pub_init_res));
    }

    //NODE 2 = AUTOPILOT 2
    uavcan::Publisher<dsdl_source::Autopilot_2> auto2_pub(node2);
    const int auto2_pub_init_res = auto2_pub.init();
    if (auto2_pub_init_res < 0)
    {
        throw std::runtime_error("Failed to start the publisher Autopilot; error: " + std::to_string(auto2_pub_init_res));
    }

    //NODE 2 = AUTOPILOT 3
    uavcan::Publisher<dsdl_source::Autopilot_3> auto3_pub(node3);
    const int auto3_pub_init_res = auto3_pub.init();
    if (auto3_pub_init_res < 0)
    {
        throw std::runtime_error("Failed to start the publisher Autopilot; error: " + std::to_string(auto3_pub_init_res));
    }

    //********************************************************************************************************************************* 
    //Priority and Timeouts (all the sime)

    //NODE 1
    auto1_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(500));
    auto1_pub.setPriority(uavcan::TransferPriority::MiddleLower);

    //NODE 2
    auto2_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(500));
    auto2_pub.setPriority(uavcan::TransferPriority::MiddleLower);

    //NODE 3
    auto3_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(500));
    auto3_pub.setPriority(uavcan::TransferPriority::MiddleLower);


    //********************************************************************************************************************************* 
    //Loading the variables

    std::vector<float> air_airspeed, air_temperature, 
                bar_altitude, bar_pressure, bar_temperature,
                gnss_hdop, gnss_latitude, gnss_longitude, gnss_altitude, gnss_speed, gnss_gcrs,
                gyro_gyrox, gyro_gyroy, gyro_gyroz, acc_accx, acc_accy, acc_accz, gyro_temp, acc_temp;


    std::vector<int>gnss_status, gnss_n_satellites, acc_gyro_rate1, acc_gyro_rate2, mag_magx, mag_magy, mag_magz;

    std::vector<bool> air_health, bar_health, gnss_health, gyro_health, acc_health, mag_health;

    std::cout << "Loading the sensor data... \n\n" << std::endl;
    std::thread thread1(load_ARSP, std::ref(air_airspeed), std::ref(air_temperature), std::ref(air_health));

    std::thread thread2(load_BAR, std::ref(bar_altitude), std::ref(bar_pressure), std::ref(bar_temperature), std::ref(bar_health));

    std::thread thread3(load_GPS, std::ref(gnss_status), std::ref(gnss_n_satellites), std::ref(gnss_hdop), std::ref(gnss_latitude),
                        std::ref(gnss_longitude), std::ref(gnss_altitude), std::ref(gnss_speed), std::ref(gnss_gcrs), std::ref(gnss_health));

    std::thread thread4(load_IMU, std::ref(gyro_gyrox), std::ref(gyro_gyroy), std::ref(gyro_gyroz), std::ref(acc_accx),
                        std::ref(acc_accy), std::ref(acc_accz), std::ref(gyro_temp), std::ref(acc_temp), std::ref(acc_gyro_rate1), std::ref(acc_gyro_rate2), 
                        std::ref(gyro_health), std::ref(acc_health));

    std::thread thread5(load_MAG, std::ref(mag_magx), std::ref(mag_magy), std::ref(mag_magz), std::ref(mag_health));

    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    thread5.join();

    //Running the nodes.
    node1.setModeOperational();
    node2.setModeOperational();
    node3.setModeOperational();

    //size for the dynamic arrays
    uint8_t number_modules = 3;
    uint8_t number_gnss = 2;
    uint8_t number_axes = 3;
    uint8_t x=0, y=1, z=2; //3 dimmensional axes

    //Defining autopilots messages and datatype sizes
    dsdl_source::Autopilot_1 auto_msg1;
    dsdl_source::Autopilot_2 auto_msg2;
    dsdl_source::Autopilot_3 auto_msg3;

    definingSystemSize(auto_msg1, number_modules, number_gnss, number_axes);
    definingSystemSize(auto_msg2, number_modules, number_gnss, number_axes);
    definingSystemSize(auto_msg3, number_modules, number_gnss, number_axes);

    auto_msg1.autopilot_index[0].autopilot = 1;
    auto_msg2.autopilot_index[0].autopilot = 2;
    auto_msg3.autopilot_index[0].autopilot = 3;

    //initial positions of the data
    uint8_t r = 0;
    uint r1;
    uint r2 = 0;
    uint r3;
    uint8_t t = 0;

    std::srand(std::time(nullptr));

    while (true)
    {
        //*********************************************************************************************************************************
        //Sorting system, thats how the data is always different

        //resting the sorted system when the files are coming to an end
        if(r2==23000)
        {
            r2=0;
            r1=0;
            r2=0;
            r=0;
            t=0;
        }

        //position of data variables
        if(r==0){
            r2 += rand() % 201 + 300; //[300, 500]
            r=1;
        }
        if(r==1){
            r2 += rand() % 10 + 1;
            t++;
            if(t>5){
                r=0;
                t=0;
            }
        }
        r1 = r2 - 300; //Autopilot 2
        r3 = r2 + 300; //Autopilot 3


        //*********************************************************************************************************************************
        //Publishing the data
                            //= node1.spin(uavcan::MonotonicDuration::fromUSec(333333)); 
        const int spin1_res = node1.spinOnce();             //instant spin
        if (spin1_res < 0)
        {
            std::cerr << "Transient 1 failure: " << spin1_res << std::endl;
        }
                            //= node2.spin(uavcan::MonotonicDuration::fromUSec(333333)); 
        const int spin2_res = node2.spinOnce();             //instant spin
        if (spin2_res < 0)
        {
            std::cerr << "Transient 2 failure: " << spin2_res << std::endl;
        }
                            //= node3.spin(uavcan::MonotonicDuration::fromUSec(333333)); 
        const int spin3_res = node3.spin(uavcan::MonotonicDuration::fromUSec(1000000));         //1 second spin
        if (spin3_res < 0)
        {
            std::cerr << "Transient 3 failure: " << spin3_res << std::endl;
        }
        //--------------------------------------------------------------------------------------------------
        //Assign Values to sensors modules
        
        //Autopilot1
        assignValues(auto_msg1, r1, x, y, z, number_modules, number_gnss, 
        air_airspeed, air_temperature, bar_altitude, bar_pressure, bar_temperature, gnss_hdop, gnss_latitude, gnss_longitude, gnss_altitude, gnss_speed, gnss_gcrs,
        gyro_gyrox, gyro_gyroy, gyro_gyroz, acc_accx, acc_accy, acc_accz, gyro_temp, acc_temp, 
        gnss_status, gnss_n_satellites, acc_gyro_rate1, acc_gyro_rate2, mag_magx, mag_magy, mag_magz, 
        air_health, bar_health, gnss_health, gyro_health, acc_health, mag_health);
        
        //Autopilot2
        assignValues(auto_msg2, r2, x, y, z, number_modules, number_gnss, 
        air_airspeed, air_temperature, bar_altitude, bar_pressure, bar_temperature, gnss_hdop, gnss_latitude, gnss_longitude, gnss_altitude, gnss_speed, gnss_gcrs,
        gyro_gyrox, gyro_gyroy, gyro_gyroz, acc_accx, acc_accy, acc_accz, gyro_temp, acc_temp, 
        gnss_status, gnss_n_satellites, acc_gyro_rate1, acc_gyro_rate2, mag_magx, mag_magy, mag_magz, 
        air_health, bar_health, gnss_health, gyro_health, acc_health, mag_health);  

        //Autopilot3
        assignValues(auto_msg3, r3, x, y, z, number_modules, number_gnss, 
        air_airspeed, air_temperature, bar_altitude, bar_pressure, bar_temperature, gnss_hdop, gnss_latitude, gnss_longitude, gnss_altitude, gnss_speed, gnss_gcrs,
        gyro_gyrox, gyro_gyroy, gyro_gyroz, acc_accx, acc_accy, acc_accz, gyro_temp, acc_temp, 
        gnss_status, gnss_n_satellites, acc_gyro_rate1, acc_gyro_rate2, mag_magx, mag_magy, mag_magz, 
        air_health, bar_health, gnss_health, gyro_health, acc_health, mag_health);      

        //--------------------------------------------------------------------------------------------------
        //Cheking Health of sensor modules
        
        /*
        checkingStatus(auto_msg1, number_modules, number_gnss);
        checkingStatus(auto_msg2, number_modules, number_gnss);
        checkingStatus(auto_msg3, number_modules, number_gnss);
        */
        //--------------------------------------------------------------------------------------------------
        //Broadcasting the message from the nodes

        const int pub_auto1_res = auto1_pub.broadcast(auto_msg1);
        if (pub_auto1_res < 0)
        {
            std::cerr << "Autopilot 1 publication failure: " << pub_auto1_res << std::endl;
        }

        const int pub_auto2_res = auto2_pub.broadcast(auto_msg2);
        if (pub_auto2_res < 0)
        {
            std::cerr << "Autopilot 2 publication failure: " << pub_auto2_res << std::endl;
        }
    
        const int pub_auto3_res = auto3_pub.broadcast(auto_msg3);
        if (pub_auto3_res < 0)
        {
            std::cerr << "Autopilot 3 publication failure: " << pub_auto3_res << std::endl;
        }

    }
}