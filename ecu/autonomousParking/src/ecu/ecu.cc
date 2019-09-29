/*
 * \brief  ECU - Receives messages from mosquitto server, processes parking manuver and publishes car control messages
 * \author Alexander Reisner
 * \date   2017-07-16
 */

/* mosquitto includes */
#include <mosquittopp.h>

/* genode includes */
#include <base/env.h>
#include <base/printf.h>
#include <util/xml_node.h>

/* lwip includes */
extern "C"
{
#include <lwip/sockets.h>
}
#include <lwip/genode.h>
#include <nic/packet_allocator.h>

/* etc */
#include <cstdio>
#include <cstring>
#include <string>
#include <timer_session/connection.h>

/*pub*/
//#include <publisher.h>
#include <subscriber.h>

/*parking*/
#include "Parking.h"

#include <base/attached_rom_dataspace.h>
#include <libc/component.h>
#include <semaphore.h>

/* Float variables that are updated on realtime by on message calls
   Needed to feed the parking algorithm with up to date values */
float steer, brake, accel, spinVel0, spinVel1, spinVel2, spinVel3, length, width, wheelRadius, gps_x, gps_y, laser0, laser1, laser2, laser3, speed, autonomous, steer_max, vel_max = 1.5, timestamp, got_go;

/* Boolean variables that are initialized false and set true, if component received a message via mosquitto
   Set to false again, after the corresponding message was used */
bool car_complete, got_length, got_width, got_wheelRadius, got_laser0, got_laser1, got_laser2, got_spinVel, go, got_steerlock;

/* The car information object is constructed once all needed information is in place and is then handed over to the paking object later on */
CarInformation *car;

/* The parking object is called every time all needed information arrived at the ECU to perform the next step of the parking manuver */
Parking *parking;

Timer::Connection timer;
uint64_t starttime, stoptime, duration, totalduration, calculationroundscounter, minval, maxval = 0;

void ecu::myPublish(const char *name, float value)
{
	char buffer[1024] = {0};
	sprintf(buffer, "%s,%f", name, value);
	/* int ret =*/publish(NULL, "car-control", strlen(buffer), buffer);
	//Genode::log("pub control '%s' successful: %d", buffer, MOSQ_ERR_SUCCESS == ret);
}

/* Message callback of mosquitto, executed every time a message on the subscribed topic arrives */
void ecu::on_message(const struct mosquitto_message *message)
{

	starttime = timer.elapsed_ms();

	//Genode::log("%s %s", message->topic, message->payload);
	std::string payload = (char *)message->payload;

	/* take first part of message until ";" and compare it to messages from protobuf
	   save second part of message into corresponding value
	   and set got_something boolean to true, if value is needed elsewhere */
	const char *name = payload.substr(0, payload.find(";")).c_str();
	if (!strcmp(name, "steer"))
	{
		payload.erase(0, payload.find(";") + 2);
		steer = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "brake"))
	{
		payload.erase(0, payload.find(";") + 2);
		brake = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "accel"))
	{
		payload.erase(0, payload.find(";") + 2);
		accel = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "wheel0"))
	{
		payload.erase(0, payload.find(";") + 2);
		spinVel0 = atof(payload.substr(0, payload.find(";")).c_str());
		got_spinVel = true;
	}

	if (!strcmp(name, "wheel1"))
	{
		payload.erase(0, payload.find(";") + 2);
		spinVel1 = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "wheel2"))
	{
		payload.erase(0, payload.find(";") + 2);
		spinVel2 = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "wheel3"))
	{
		payload.erase(0, payload.find(";") + 2);
		spinVel3 = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "length"))
	{
		payload.erase(0, payload.find(";") + 2);
		length = atof(payload.substr(0, payload.find(";")).c_str());
		got_length = true;
	}

	if (!strcmp(name, "width"))
	{
		payload.erase(0, payload.find(";") + 2);
		width = atof(payload.substr(0, payload.find(";")).c_str());
		got_width = true;
	}

	if (!strcmp(name, "wheelRadius"))
	{
		payload.erase(0, payload.find(";") + 2);
		wheelRadius = atof(payload.substr(0, payload.find(";")).c_str());
		got_wheelRadius = true;
	}

	if (!strcmp(name, "gps_x"))
	{
		payload.erase(0, payload.find(";") + 2);
		gps_x = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "gps_y"))
	{
		payload.erase(0, payload.find(";") + 2);
		gps_y = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "laser0"))
	{
		payload.erase(0, payload.find(";") + 2);
		laser0 = atof(payload.substr(0, payload.find(";")).c_str());
		got_laser0 = true;
	}

	if (!strcmp(name, "laser1"))
	{
		payload.erase(0, payload.find(";") + 2);
		laser1 = atof(payload.substr(0, payload.find(";")).c_str());
		got_laser1 = true;
	}

	if (!strcmp(name, "laser2"))
	{
		payload.erase(0, payload.find(";") + 2);
		laser2 = atof(payload.substr(0, payload.find(";")).c_str());
		got_laser2 = true;
	}

	if (!strcmp(name, "steerlock"))
	{
		payload.erase(0, payload.find(";") + 2);
		steer_max = atof(payload.substr(0, payload.find(";")).c_str());
		got_steerlock = true;
	}

	if (!strcmp(name, "timestamp"))
	{
		payload.erase(0, payload.find(";") + 2);
		timestamp = atof(payload.substr(0, payload.find(";")).c_str());
	}

	if (!strcmp(name, "go"))
	{
		payload.erase(0, payload.find(";") + 2);
		got_go = atof(payload.substr(0, payload.find(";")).c_str());
		/* protobuf publishes a float for go,
		   which has to become a boolean here */
		if (got_go > 0)
		{
			go = true;
			myPublish("3", 1);
		}
		else
		{
			go = false;
			myPublish("3", 0);
		}
	}

	/* if car information was already constructed, do not do it again */
	if (!car_complete)
	{
		if (got_length && got_width && got_wheelRadius && got_steerlock)
		{
			car = new CarInformation(length, width, wheelRadius, steer_max, vel_max);
			car_complete = true;
			parking = new Parking(*car);
		}
	}

	/* if all information necessary for next parking calculation arrived
	   and autonomous parking is desired, let the algorithm do its thing */
	if (go && got_laser0 && got_laser1 && got_laser2 && got_spinVel)
	{
		//Genode::log("laser0: ", laser0, " laser1: ",laser1," laser2: ", laser2, " laser3: " ,laser3 , " spinvel: ",spinVel0);
		parking->receiveData(laser0, laser1, laser2, spinVel0, timestamp, *this);
		got_laser0 = false;
		got_laser1 = false;
		got_laser2 = false;
		got_spinVel = false;

		stoptime = timer.elapsed_ms();
		duration = stoptime - starttime;
		totalduration += duration;
		calculationroundscounter++;

		if (calculationroundscounter == 1)
		{
			minval = duration;
		}
		minval = std::min(minval, duration);
		maxval = std::max(maxval, duration);

		if (calculationroundscounter % 500 == 0)
		{
			Genode::log("The duration for calculation and sending was ", duration, " milliseconds");
			Genode::log("The TOTALduration for calculation and sending was ", totalduration, " milliseconds");
			Genode::log("The MINduration for calculation and sending was ", minval, " milliseconds");
			Genode::log("The MAXduration for calculation and sending was ", maxval, " milliseconds");
			Genode::log("The AVERAGEduration for calculation and sending was ", (totalduration / calculationroundscounter), " milliseconds after ", calculationroundscounter, " steps");
		}
	}
}

void ecu::on_connect(int rc)
{
	Genode::log("connected to mosquitto server");
}

void ecu::on_disconnect(int rc)
{
	Genode::log("disconnect from mosquitto server");
}

ecu::ecu(const char *id, Libc::Env &_env) : mosquittopp(id)
{
	//lwip_tcpip_init(); /* causes freeze, code works fine without it */

	/* network initialization... */

	/* initialization */
	sem_init(&allValSem, 0, 1);
	sem_init(&allData, 0, 0);
	mosqpp::lib_init();

	Genode::Attached_rom_dataspace _config(_env, "config");
	/* configure mosquitto library */
	Genode::Xml_node mosquitto = _config.xml().sub_node("mosquitto");
	try {
		mosquitto.attribute("ip-address").value(this->host, sizeof(host));
	} catch(Genode::Xml_node::Nonexistent_attribute) {
		Genode::error("mosquitto ip-address is missing from config");
	}
	this->port = mosquitto.attribute_value<unsigned int>("port", 1883);
	this->keepalive = mosquitto.attribute_value<unsigned int>("keepalive", 60);

	/* connect to mosquitto server */
	
	int ret;
	do {
		ret = this->connect(host, port, keepalive);
		Genode::log("I am alive!");
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto connect");
			return;
		case MOSQ_ERR_ERRNO:
			Genode::log("mosquitto ", (const char *)strerror(errno));
		default:
			timer.msleep(1000);
			break;
		}
	} while(ret != MOSQ_ERR_SUCCESS);
	Genode::log("I am connected to mosquittoooo!");
	
	/* subscribe to topic */
	do {
		ret = this->subscribe(NULL, topic);
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto subscribe");
			return;
		case MOSQ_ERR_NOMEM:
			Genode::error("out of memory condition occurred");
			return;
		case MOSQ_ERR_NO_CONN:
			Genode::error("not connected to a broker");
			return;
		}
	} while(ret != MOSQ_ERR_SUCCESS);



	/* initiate any value with false, you never know... */
	got_laser0 = false;
	got_laser1 = false;
	got_laser2 = false;
	got_spinVel = false;
	car_complete = false;
	got_length = false;
	got_width = false;
	got_wheelRadius = false;
	go = false;

	Genode::log("done");

	/* start non-blocking loop */
	ret = loop_start();
	if (ret != MOSQ_ERR_SUCCESS) {
		switch(ret) {
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto loop_start");
			return;
		case MOSQ_ERR_NOT_SUPPORTED:
			Genode::error("mosquitto no thread support");
			return;
		}
	}
	


	/* loop_start creates a thread and makes it possible to execute code afterwards
	   loop_forever creates a thread and lets no code run afterwards */
	/* cleanup */
	//mosqpp::lib_cleanup();
}
/* TODO */
ecu::~ecu()
{
}

/* cleanup */
//	mosqpp::lib_cleanup();
//}