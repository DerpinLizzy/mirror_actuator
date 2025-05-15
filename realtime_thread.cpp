#include "realtime_thread.h"
#include <cstdint>
using namespace std;

extern GPA myGPA;
extern DataLogger myDataLogger;

// contructor for controller loop
realtime_thread::realtime_thread(Data_Xchange *data,IO_handler *io, Mirror_Kinematic *mk, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_data = data;        // link to data
    this->m_io = io;            // link to hardware
    this->m_mk = mk;            // link to kinematics
    ti.reset();
    ti.start();
    controller_state = CNTRL_IDLE;  // the local state machine
    // v_cntrl_0 = PID_Cntrl(0.0209,4.18,0,0,Ts,-0.8,0.8); // zunaechst nur PI-Regler, 1 
    // v_cntrl_1 = PID_Cntrl(0.0209,4.18,0,0,Ts,-0.8,0.8); // zunaechst nur PI-Regler, 1 
    v_cntrl_0 = PID_Cntrl(0.0121, 1.66, 1.78e-5, 0.00025, Ts, -0.8, 0.8); // PID Regler vollständig
    v_cntrl_1 = PID_Cntrl(0.0121, 1.66, 1.78e-5, 0.00025, Ts, -0.8, 0.8); // PID Regler vollständig

    // Ableitungsfilter für Vorsteuerung
    ableit_vorst0 = IIR_filter(2*Ts, Ts);
    ableit_vorst1 = IIR_filter(2*Ts, Ts);

    }
// decontructor for controller loop
realtime_thread::~realtime_thread() {}
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void realtime_thread::loop(void){
    float i_des0,i_des1,v_des,phi_des,v_des_vorst;
    uint8_t k = 0;
    float kv = 0;
    float kp = .02;
    float amp = 50;
    float omega = 2 * 3.1415 * 20;

    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_io->read_encoders_calc_speed();       // first read encoders and calculate speed
        // -------------------------------------------------------------
        // at very beginning: move system slowly to find the zero pulse
        float ti_loc = ti.read();
        m_data->cntrl_xy_des[0] = amp * cosf(omega * ti_loc);
        m_data->cntrl_xy_des[1] = amp * sinf(omega * ti_loc);
        m_mk->X2P(m_data->cntrl_xy_des, m_data->cntrl_phi_des);

        switch(controller_state)
            {
            case CNTRL_IDLE:
                i_des0 = i_des1 = 0;
                break;
            case FIND_INDEX:
                // Aufgabe 8.x
                i_des0 = v_cntrl_0(1 - m_data->sens_Vphi[0]);
                i_des1 = v_cntrl_1(1 - m_data->sens_Vphi[1]);
                m_io->enable_motors(true);      // enable motors, still read the bigButton to enable
                break;

            case GPA_IDENT_PLANT:
                m_io->enable_motors(true);      // enable motors, still read the bigButton to enable
                // i_des0 = myGPA.update(i_des0, m_data->sens_Vphi[0]); // open loop
                // i_des0 = 0.02*(80 + myGPA.update( i_des0, m_data->sens_Vphi[1]) - m_data->sens_Vphi[1]); // closed loop
                // i_des0 = 0.02*(80 - m_data->sens_Vphi[1]) + myGPA.update(i_des0, m_data->sens_Vphi[1]); // Variante 3

                i_des0 = v_cntrl_0(v_des - m_data->sens_Vphi[0]);
                v_des = myGPA.update(v_des, m_data->sens_phi[0]);

                i_des1 = 0;
                break;

            case CNTRL_VEL:
                v_des = myDataLogger.get_set_value(ti_loc);
                i_des0 = v_cntrl_0(v_des - m_data->sens_Vphi[0]);
                i_des1 = v_cntrl_1(10 - m_data->sens_Vphi[1]);
                m_io->enable_motors(true);      // enable motors
                myDataLogger.write_to_log(ti_loc, v_des, m_data->sens_Vphi[0], i_des0);
                break;

            case CNTRL_POS:
                m_io->enable_motors(true);

                // Winkelregler
                // phi_des = myDataLogger.get_set_value(ti_loc);
                phi_des = m_data->cntrl_phi_des[0];
                v_des = kv * phi_des - m_data->sens_phi[0] + ableit_vorst0(phi_des);
                i_des0 = v_cntrl_0(v_des - m_data->sens_Vphi[0]);
                myDataLogger.write_to_log(ti_loc, phi_des, m_data->sens_phi[0], i_des0);

                phi_des = m_data->cntrl_phi_des[1];
                v_des = kv * phi_des - m_data->sens_phi[1] + ableit_vorst1(phi_des);
                i_des1 = v_cntrl_1(v_des - m_data->sens_Vphi[1]);

                break;
            // ------------------------ do the control first
            default:
                break;
            }
        m_io->write_current(0,i_des0);
        m_io->write_current(1,i_des1);       // set 2nd motor to 0A
        m_io->set_laser_on_off(m_data->laser_on);
        if(++k>=10)     // kinematic transformation from angles to xy values only every 10th time.
            {
            m_mk->P2X(m_data->sens_phi,m_data->est_xy);
            k = 0;
            }
            
        }// endof the main loop
}

void realtime_thread::sendSignal() {
    thread.flags_set(threadFlag);
}
void realtime_thread::start_loop(void)
{
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), Ts);
}
// several public functions to allow the controller statemachine to switch 
// to other states from external.
void realtime_thread::switch_to_find_index()
{
    controller_state = FIND_INDEX;
}
void realtime_thread::switch_to_GPA_ident()
{
    controller_state = GPA_IDENT_PLANT;
}
void realtime_thread::switch_to_cntrl_vel()
{
    controller_state = CNTRL_VEL;
}
void realtime_thread::switch_to_cntrl_pos()
{
    controller_state = CNTRL_POS;
}
void realtime_thread::init_controllers(void)
{
    // set values for your velocity and position controller here!
}
   
void realtime_thread::reset_pids(void)
{
    // reset all cntrls.
}