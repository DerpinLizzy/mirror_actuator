#include "IO_handler.h"

#define PI 3.1415927
// constructors


// Deconstructor
IO_handler::IO_handler(Data_Xchange *data,Mirror_Kinematic *mk, float Ts) : di1(.0005,Ts),di2(.0005,Ts),big_button(PC_3),counter1(PA_6, PC_7),
                            indexpulse1(PA_8),index1(counter1,indexpulse1), counter2(PB_6, PB_7),indexpulse2(PB_4),index2(counter2,indexpulse2),
                            i_enable(PC_4),laser_on(PB_0), i_des1(PA_5),i_des2(PA_4)
{
    this->m_data = data;
    this->m_mk = mk;
    i2u.setup(-.80,.80,0.0f,1.0f);
    uw1 = Enc_unwrap_scale(4000,16);
    uw2 = Enc_unwrap_scale(4000,16);
    i_enable = 0;       // disable current first
    counter1.reset();   // encoder reset
    counter2.reset();   // encoder reset
    this->set_laser_on_off(false);
}
IO_handler::~IO_handler() {} 

void IO_handler::read_encoders_calc_speed(void)
{
    m_data->sens_phi[0] = uw1(counter1);
    m_data->sens_phi[1] = uw2(counter2);
    m_data->sens_Vphi[0] = di1(m_data->sens_phi[0]);
    m_data->sens_Vphi[1] = di2(m_data->sens_phi[1]);
}

void IO_handler::enable_motors(bool enable)
{
    // i_enable = big_button && enable;    
    i_enable = enable;    
}
void IO_handler::force_enable_motors(bool enable)
{
    i_enable = enable;    
}

void IO_handler::write_current(uint8_t mot_nb, float i_des)
{
    if(mot_nb== 0)
        i_des1 = i2u(i_des);
    else if(mot_nb == 1)
        i_des2 = i2u(i_des);    
}
void IO_handler::set_laser_on_off(bool laser_on_off)
{
    laser_on = laser_on_off;
}
bool IO_handler::motors_are_referenced()
{
    return (index1.is_referenced && index2.is_referenced);
}