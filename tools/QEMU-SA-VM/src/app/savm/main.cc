/*
 * \brief  sa/vm - receives tcp/ip packets with protobuf format and publishes mosquitto messages to server
 * \author Alexander Reisner
 * \date   2017-07-16
 */

/* protobuf include */
#include <state.pb.h>
#include <control.pb.h>

#define timeval timeval_linux

#include "publisher.h"
#include "subscriber.h"
#include "proto_client.h"

/* lwip includes */
extern "C" {
#include <lwip/sockets.h>
}
#include <lwip/genode.h>
#include <nic/packet_allocator.h>

/* genode includes */
#include <base/env.h>
#include <base/printf.h>
#include <util/xml_node.h>
#include <os/config.h>
#include <ram_session/connection.h>
#include <timer_session/connection.h>

/* etc */
#include <cstdio>
#include <cstring>

/* Float variables are used to store the last step of parking calculation.
   If no updated value arrives at SAVM before SD2 requires a new car control protobuf message,
   the "old" values are reused. */
float steer, brake, accel, speed;

/* Boolean value is used to store state of driving */
bool autonomous;


Timer::Connection timer;
uint64_t starttime,stoptime,duration,totalduration,calculationroundscounter,minval,maxval = 0;
uint64_t CONNstarttime,CONNstoptime,CONNduration,CONNtotalduration,CONNcalculationroundscounter,CONNminval,CONNmaxval = 0;

void Publisher::my_publish(const char* name, float value) {
	char buffer[1024] = { 0 };
	sprintf(buffer, "%s; %f;", name, value);
	/* int ret = */ publish(NULL, "state", strlen(buffer), buffer);
	//Genode::log("pub state '%s' successful: %d", buffer, MOSQ_ERR_SUCCESS == ret);
}

/* Receive car control messages from server and store them in corresponding values locally.
   The values are then forwared to SD2. */
void Subscriber::on_message(const struct mosquitto_message *message) {
	//Genode::log("%s %s", message->topic, message->payload);

	starttime = timer.elapsed_ms();

	/* Take first part of message, alias identifier to decide which value arrived */
	std::string payload = (char*)message->payload;
	const char* name = payload.substr(0, payload.find(",")).c_str();

	if(!strcmp(name,"0"))
	{
		payload.erase(0, payload.find(",")+1);
		steer=atof(payload.c_str());
	}
	if(!strcmp(name,"1"))
	{
		payload.erase(0, payload.find(",")+1);
		brake=atof(payload.c_str());

	}
	if(!strcmp(name,"2"))
	{
		payload.erase(0, payload.find(",")+1);
		accel=atof(payload.c_str());

	}
	if(!strcmp(name,"3"))
	{
		payload.erase(0, payload.find(",")+1);
		float tmp=atof(payload.c_str());
		if(tmp>0)
		{
			autonomous=true;
		}
		else
		{
			autonomous=false;
		}
	}
	if(!strcmp(name,"4"))
	{
		payload.erase(0, payload.find(",")+1);
		speed=atof(payload.c_str());
	}

	stoptime = timer.elapsed_ms();
	duration = stoptime - starttime;
	totalduration += duration;
	calculationroundscounter++;

	if (calculationroundscounter == 1){
		minval = duration;
	}
	minval = std::min(minval,duration);
	maxval = std::max(maxval,duration);

	if(calculationroundscounter % 500 == 0){
	Genode::log("The duration for calculation and sending was ", duration ," milliseconds");
	Genode::log("The TOTALduration for calculation and sending was ", totalduration ," milliseconds");
	Genode::log("The MINduration for calculation and sending was ", minval ," milliseconds");
	Genode::log("The MAXduration for calculation and sending was ", maxval ," milliseconds");
	Genode::log("The AVERAGEduration for calculation and sending was ", (totalduration / calculationroundscounter) ," milliseconds after ", calculationroundscounter, " steps");
	}
}

Proto_client::Proto_client() :
	_listen_socket(0),
	_in_addr{0},
	_target_addr{0}
{

	/* Init TCP/IP connection with SD2.
	   Google Protobuf is used as a protocol. */
	enum { BUF_SIZE = Nic::Packet_allocator::DEFAULT_PACKET_SIZE * 128 };

	/* Get values from run file */
	Genode::Xml_node speedDreams = Genode::config()->xml_node().sub_node("speedDreams");

	char ip_addr[16] = {0};
	char port[5] = {0};

	speedDreams.attribute("ip-address").value(ip_addr, sizeof(ip_addr));
	speedDreams.attribute("port").value(port, sizeof(port));

	_in_addr.sin_addr.s_addr = inet_addr(ip_addr);
	_in_addr.sin_family = AF_INET;
	_in_addr.sin_port = htons(atoi(port));

	if ((_listen_socket = lwip_socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		PERR("No socket available!");
		return;
	}
	Genode::log("listen socket\n");

	if (lwip_connect(_listen_socket, (struct sockaddr*)&_in_addr, sizeof(_in_addr)))
	{
		PERR("Connection failed!");
		return;
	}

	PINF("Connected...\n");
	int yes = 1;
	/* Make lwip send message immediately, with NODELAY */
   	int result = lwip_setsockopt(_listen_socket, IPPROTO_TCP, TCP_NODELAY, (char *) &yes, sizeof(int));



}

Proto_client::~Proto_client()
{
	
}

void Proto_client::serve(Publisher *publisher)
{
	/* Init size of protobuf message with 0 */
	int size = 0;
	/* Allocate ram dataspace where protobuf message will be stored */
	Genode::Ram_dataspace_capability state_ds=Genode::env()->ram_session()->alloc(1024);
	/* Attach dataspace to our rm session */
	char* bar=Genode::env()->rm_session()->attach(state_ds);
	while (true)
	{
		CONNstarttime = timer.elapsed_ms();

		/* Set size to 0 are every loop */
		size=0;
		/* Get size of message from SD2 */
		lwip_read(_listen_socket, &size, ntohl(4));
		if (size>0)
		{
			/* Receive message of told size from SD2 */
			lwip_read(_listen_socket, bar, ntohl(size));
			protobuf::State state;
			/* Deserialize from char* to state */
			state.ParseFromArray(bar,ntohl(size));
			/* Publish any value we got from SD2 */		
			float steerCmd=state.steer();
			publisher->my_publish("steer",steerCmd);
			float brakeCmd=state.brakecmd();
			publisher->my_publish("brake",brakeCmd);		
			float accelCmd=state.accelcmd();
			publisher->my_publish("accel",accelCmd);
			float spinVel0=state.wheel(0).spinvel();
			publisher->my_publish("wheel0",spinVel0);
			float spinVel1=state.wheel(1).spinvel();
			publisher->my_publish("wheel1",spinVel1);
			float spinVel2=state.wheel(2).spinvel();
			publisher->my_publish("wheel2",spinVel2);
			float spinVel3=state.wheel(3).spinvel();
			publisher->my_publish("wheel3",spinVel3);
			float length=state.specification().length();
			publisher->my_publish("length",length);
			float width=state.specification().width();
			publisher->my_publish("width",width);
			float wheelRadius=state.specification().wheelradius();
			publisher->my_publish("wheelRadius",wheelRadius);	
			float steerlock=state.specification().steerlock();
			publisher->my_publish("steerlock",steerlock);
			float timestamp=state.timestamp();
			publisher->my_publish("timestamp",timestamp);
			/* Go through sensors */
			for(int i=0; i<state.sensor().size(); i++)
			{
				/* GPS sensor */
				if(state.sensor(i).type()==protobuf::Sensor_SensorType_GPS)
				{
					float gps_x=state.sensor(i).value(0);
					publisher->my_publish("gps_x",gps_x);
					float gps_y=state.sensor(i).value(1);
					publisher->my_publish("gps_y",gps_y);
				}
				/* Laser sensor */
				else
				{
					float laser=state.sensor(i).value(0);
					char buffer[1024] = { 0 };
					sprintf(buffer, "laser%d", i);
					const char* name=buffer;
					publisher->my_publish(name,laser);
				}
			}
			/* String control data is serialized to */
			std::string foo;
			protobuf::Control ctrl;
			/* Set values received from ECU/Parking to control */
			ctrl.set_steer(steer);
			ctrl.set_accelcmd(accel);
			ctrl.set_brakecmd(brake);
			ctrl.set_speed(speed);
			ctrl.set_autonomous(autonomous);
			/* Serialize control to String */
			ctrl.SerializeToString(&foo);
			/* Get size of serialized String */
			int32_t m_length=foo.size();
			/* Send size of serialized String to SD2 */
			lwip_write(_listen_socket,&m_length,4);
			/* Send serialized String to SD2 */
			lwip_write(_listen_socket,(void*)foo.c_str(),foo.size());
		}
		else
		{
			//PWRN("Unknown message: %d", size);
		}

	CONNstoptime = timer.elapsed_ms();
	CONNduration = CONNstoptime - CONNstarttime;
	CONNtotalduration += CONNduration;
	CONNcalculationroundscounter++;

	if (CONNcalculationroundscounter == 1){
		CONNminval = CONNduration;
	}
	CONNminval = std::min(CONNminval,CONNduration);
	CONNmaxval = std::max(CONNmaxval,CONNduration);

	if(CONNcalculationroundscounter % 500 == 0){
	Genode::log("The CONNduration for calculation and sending was ", CONNduration ," milliseconds");
	Genode::log("The CONNTOTALduration for calculation and sending was ", CONNtotalduration ," milliseconds");
	Genode::log("The CONNMINduration for calculation and sending was ", CONNminval ," milliseconds");
	Genode::log("The CONNMAXduration for calculation and sending was ", CONNmaxval ," milliseconds");
	Genode::log("The CONNAVERAGEduration for calculation and sending was ", (CONNtotalduration / CONNcalculationroundscounter) ," milliseconds after ", CONNcalculationroundscounter, " steps");
	}

	}
	Genode::env()->ram_session()->free(state_ds);
}

int Proto_client::connect()
{
	return 0;
}

void Proto_client::disconnect()
{
	lwip_close(_listen_socket);
	PERR("Server socket closed.");
}	


int main(int argc, char* argv[]) {
	//lwip_tcpip_init(); /* causes freeze, code works fine without it */

	enum { BUF_SIZE = Nic::Packet_allocator::DEFAULT_PACKET_SIZE * 128 };

	Genode::Xml_node network = Genode::config()->xml_node().sub_node("network");

	/* Init network ... */
	if (network.attribute_value<bool>("dhcp", true)) {
		Genode::log("DHCP network...");
		if (lwip_nic_init(0,
		                  0,
		                  0,
		                  BUF_SIZE,
		                  BUF_SIZE)) {
			PERR("lwip init failed!");
			return 1;
		}
		/* Wait for DHCP IP assignement */
		Genode::log("waiting 10s for dhcp ip");
		Timer::Connection timer;
		timer.msleep(10000);
		Genode::log("done");
	} else {
		Genode::log("manual network...");
		char ip_addr[16] = {0};
		char subnet[16] = {0};
		char gateway[16] = {0};

		network.attribute("ip-address").value(ip_addr, sizeof(ip_addr));
		network.attribute("subnet-mask").value(subnet, sizeof(subnet));
		network.attribute("default-gateway").value(gateway, sizeof(gateway));

		if (lwip_nic_init(inet_addr(ip_addr),
		                  inet_addr(subnet),
		                  inet_addr(gateway),
		                  BUF_SIZE,
		                  BUF_SIZE)) {
			PERR("lwip init failed!");
			return 1;
		}
	}

	/* get config */
	Genode::Xml_node mosquitto = Genode::config()->xml_node().sub_node("mosquitto");

	char ip_addr[16] = {0};
	char port[5] = {0};

	mosquitto.attribute("ip-address").value(ip_addr, sizeof(ip_addr));
	mosquitto.attribute("port").value(port, sizeof(port));

	/* create TCP/IP protobuf connection to and from SD2 */
	Genode::log("protobuf init");
	Proto_client *client = new Proto_client();
	Genode::log("done");

	/* create SAVM publisher */
	Genode::log("savm pub init");
	Publisher *pub = new Publisher("SAVMPub", ip_addr, atoi(port));
	Genode::log("done");
	
	/* create SAVM subscriber */
	Genode::log("savm sub init");
	Subscriber *sub = new Subscriber("SAVMSub", ip_addr, atoi(port));
	sub->my_subscribe("car-control");
	Genode::log("done");

	/* endless loop with auto reconnect */
	pub->loop_start();
	sub->loop_start();

	/* use loop_start instead of loop_forever to be able to run line below */
	/* start TCP/IP loop */
	client->serve(pub);

	/* cleanup */
	mosqpp::lib_cleanup();
	return 0;
}
