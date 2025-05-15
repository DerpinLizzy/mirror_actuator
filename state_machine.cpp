#include "state_machine.h"
using namespace std;

extern GPA myGPA;

// contructor for controller loop
state_machine::state_machine(IO_handler *io, realtime_thread *loop, float Ts) : thread(osPriorityNormal,4096)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_io = io;
    this->m_loop = loop;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
state_machine::~state_machine() {}

// ----------------------------------------------------------------------------
void state_machine::loop(void){
    
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        switch(CS)
            {
            case INIT:
                if(ti.read()>1)
                    {
                    ti.reset();
                    //m_loop->switch_to_GPA_ident();
                    // m_loop->switch_to_cntrl_vel();
                    m_loop->switch_to_cntrl_pos();
                    CS = CONTROL;
                    }
                break;
            case REFERENCE:
                if(m_io->motors_are_referenced())
                    {
                    ti.reset();
                    m_loop->switch_to_cntrl_vel();
                    CS = STATE_GPA;
                    }
                break;
            case STATE_GPA:
                break;
            case CONTROL:
                break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}
