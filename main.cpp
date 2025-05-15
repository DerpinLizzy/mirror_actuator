#include "mbed.h"
#include <stdint.h>
#include "math.h" 
#include "GPA.h"
#include "DataLogger.h"
#include "realtime_thread.h"
#include "Mirror_Kinematic.h"
#include "data_structs.h"
#include "FastPWM.h"
#include "IO_handler.h"
#include "uart_comm_thread_send.h"
#include "uart_comm_thread_receive.h"
#include "state_machine.h"
 
float Ts = 0.0002f;                    // sampling time
// --------- local functions
void reset_data(Data_Xchange *);
//----------------------------------------- global variables (uhh!) ---------------------------
//init values:    (f0,   f1, nbPts, A0, A1, Ts)
GPA          myGPA(5, 1000, 30, 1,1, Ts);
DataLogger   myDataLogger(1);

//******************************************************************************
//---------- main loop -------------
//******************************************************************************
int main()
{
    // --------- Mirror kinematik, define values, trafos etc there
    Data_Xchange data;              // data exchange structure, see data_structs.h in the "Lib_Misc" library
    Mirror_Kinematic mk(&data);     // Mirror_Kinematics class, the geom. parameters, trafos etc. are done
    IO_handler hardware(&data,&mk,Ts);
    static BufferedSerial uart_serial(USBTX, USBRX, 115200);
    uart_serial.set_format(8,BufferedSerial::None,1);
    uart_serial.set_blocking(false); // force to send whenever possible and data is there
    uart_comm_thread_send uart_com_send(&data,&hardware,&uart_serial, .02f); // communication send thread
    uart_comm_thread_receive uart_com_receive(&data,&mk,&uart_serial, .01f); // communication receive thread
    realtime_thread loop(&data,&hardware,&mk,Ts);       // this is for the main controller loop
    state_machine sm(&hardware,&loop,.01);              // handles states
    reset_data(&data);
    ThisThread::sleep_for(200);
// ----------------------------------
    mk.set_offsets(0,0);          // individal set values for global position
    mk.trafo_is_on =  true;
    loop.init_controllers();
    uart_com_receive.start_uart();
    uart_com_send.start_uart();
    loop.start_loop();
    sm.start_loop();
    while(1)
        ThisThread::sleep_for(200);
     
}   // END OF main


void reset_data(Data_Xchange *da)
{
    for(uint8_t k=0;k<2;k++)
        {
        da->sens_phi[k] = 0;
        da->sens_Vphi[k] = 0;
        da->est_xy[k] = 0;
        da->cntrl_phi_des[k] = 0;
        da->cntrl_Vphi_des[k] = 0;
        da->cntrl_xy_des[k] = 0;
        da->i_des[k] = 0;        
        da->wMot[k] = 0;         
        }
    da->laser_on = false;
    da->num_it = 0;
} 
