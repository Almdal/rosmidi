// Initial version of midiinterface
// Purpose: Read USB midi device and publish messages on topics


#include <iostream>
#include <cstdlib>
#include "RtMidi.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include "midiinterface/MidiControlMessage.h"
#include "midiinterface/MidiProgramChangeMessage.h"

using std::cout;
using std::endl;

void midiCallback( double timeStamp, std::vector<unsigned char> *message, void *userData);


void Callback(const ros::TimerEvent& event)
{
    ROS_DEBUG("Timer event\n");
    std::cout << "Timer event" << std::endl;
}

struct MidiHandler{
    MidiHandler(const ros::NodeHandle& nh, RtMidiIn* pMidiIn):
        m_nh(nh), m_pMidiIn(pMidiIn)
    {
        m_controlMessagePub = m_nh.advertise<midiinterface::MidiControlMessage>("midi_control_message", 10);
        m_programChangeMessagePub = m_nh.advertise<midiinterface::MidiProgramChangeMessage>("midi_program_change_message", 10);
        SetupMidiDevice();
        cout << "MidiHandler created" << endl;
    }

    void SetupMidiDevice()
    {
        if(m_pMidiIn)
        {
            if(m_pMidiIn->getPortCount())
            {
                for(unsigned int i = 0; i < m_pMidiIn->getPortCount(); i++)
                {
                    std::string name = m_pMidiIn->getPortName(i);
                    if(name.find("CH345") != std::string::npos){
                        cout << "CH345 interface found on port " << i << endl;
                        cout << "Opening port " << i << endl;
                        m_pMidiIn->openPort(i);
                        m_pMidiIn->setCallback(&midiCallback, this);
                        // Ignore sysex, timing, and active sensing messages.
                        m_pMidiIn->ignoreTypes( false, false, false);
                        break;
                    }
                }
            }
        }
    }

    void HandleMidiData( double timeStamp, std::vector<unsigned char> *message)
    {
        if(!message->size()) // Should never happen
            return;
        // https://www.midi.org/specifications-old/item/table-1-summary-of-midi-message
        unsigned char status = (message->at(0)>>4)&0x0F;

		bool print = false;

        switch(status){
        case 0x0C:
            cout << "Received Program change message" << endl;
            if(message->size()>1){
                midiinterface::MidiProgramChangeMessage msg;
                msg.header.stamp = ros::Time::now();
                msg.program = message->at(1);
                m_programChangeMessagePub.publish(msg);
            }
            print = true;
            break;
        case 0x0F:
			//cout << "SysEx message" << endl;
			if(message->size() < 4) break;
			if(message->at(1) == 0x7F && message->at(3) == 6)
				cout << "MMC Command" << endl;
			print = true;
			break;
        case 0x0B:
        {
            cout << "Reveived Control Change message " << endl;
            midiinterface::MidiControlMessage msg;
            msg.header.stamp = ros::Time::now();
            msg.channel = message->at(1);
            msg.value = message->at(2);
            m_controlMessagePub.publish(msg);
            print = true;
		}
            break;
            
        default:
			break;
            //cout << "Unknown Midi header type " << (int) message->at(0) << endl;
        }
        
        if(print)
        {
			for(size_t i = 0; i < message->size(); i++)
				printf("0x%02x ", message->at(i));
			printf("\n");
		}
			
    }

    ros::Publisher m_controlMessagePub;
    ros::Publisher m_programChangeMessagePub;

    ros::NodeHandle m_nh;
    RtMidiIn* m_pMidiIn;

};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "midiinterface");
    ros::NodeHandle nh;

    //	ros::Timer timer1 = nh.createTimer(ros::Duration(1.0), Callback);

    RtMidiIn  *midiin = 0;
    uint32_t midiPorts = 0;

    do{
        // RtMidiIn constructor
        try {
            midiin = new RtMidiIn();
        }
        catch ( RtMidiError &error ) {
            error.printMessage();
            cout << "No midi ports found. Retrying in 10 seconds." << endl;
            ros::Duration(10).sleep();
        }
    } while (midiin == 0);
    cout << "MidiIn created" << endl;

    MidiHandler midiHandler(nh, midiin);







    ros::spin();
    delete midiin;
    exit(0);




    RtMidiOut *midiout = 0;
    // Check inputs.
    unsigned int nPorts = midiin->getPortCount();
    std::cout << "\nThere are " << nPorts << " MIDI input sources available.\n";
    std::string portName;
    for ( unsigned int i=0; i<nPorts; i++ ) {
        try {
            portName = midiin->getPortName(i);
        }
        catch ( RtMidiError &error ) {
            error.printMessage();
            goto cleanup;
        }
        std::cout << "  Input Port #" << i+1 << ": " << portName << '\n';
    }
    // RtMidiOut constructor
    try {
        midiout = new RtMidiOut();
    }
    catch ( RtMidiError &error ) {
        error.printMessage();
        exit( EXIT_FAILURE );
    }
    // Check outputs.
    nPorts = midiout->getPortCount();
    std::cout << "\nThere are " << nPorts << " MIDI output ports available.\n";
    for ( unsigned int i=0; i<nPorts; i++ ) {
        try {
            portName = midiout->getPortName(i);
        }
        catch (RtMidiError &error) {
            error.printMessage();
            goto cleanup;
        }
        std::cout << "  Output Port #" << i+1 << ": " << portName << '\n';
    }
    std::cout << '\n';
    // Clean up
cleanup:
    delete midiin;
    delete midiout;
    return 0;
}

// Hackish way of confining data handling to MidiHandler
void midiCallback( double timeStamp, std::vector<unsigned char> *message, void *userData){
    MidiHandler* pmh = (MidiHandler*) userData;
    pmh->HandleMidiData(timeStamp, message);
}
