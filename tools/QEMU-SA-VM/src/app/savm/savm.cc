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
extern "C"
{
#include <lwip/sockets.h>
#include <lwip/api.h>
}
#include <lwip/genode.h>
#include <nic/packet_allocator.h>

/* genode includes */
#include <base/env.h>
#include <base/printf.h>
#include <util/xml_node.h>

#include <pd_session/connection.h>
#include <timer_session/connection.h>

/* etc */
#include <cstdio>
#include <cstring>

#include <base/attached_rom_dataspace.h>
#include <libc/component.h>
#include <base/thread.h>
#include <semaphore.h>

#include <savm/savm.h>
#include <base/log.h>
#include <string.h>
#include <errno.h>

/* Float variables are used to store the last step of parking calculation.
   If no updated value arrives at SAVM before SD2 requires a new car control protobuf message,
   the "old" values are reused. */
float steer, brake, accel, speed;

/* Boolean value is used to store state of driving */
bool autonomous;

Timer::Connection timer;
uint64_t starttime, stoptime, duration, totalduration, calculationroundscounter, minval, maxval = 0;
uint64_t CONNstarttime, CONNstoptime, CONNduration, CONNtotalduration, CONNcalculationroundscounter, CONNminval, CONNmaxval = 0;

sem_t allValSem;
sem_t allData;
int allValues;

void savm::readAllBytes(void *buf, int socket, unsigned int size)
{
	unsigned int offset = 0;
	int ret = 0;

	do
	{
		ret = lwip_read(socket, (char *)buf + offset, size - offset);
		if (ret == -1)
		{
			Genode::error("lwip_read failed: ", (const char *)strerror(errno));
		}
		else
		{
			offset += ret;
		}
	} while (offset != size);
}

void savm::myPublish(const char *type, const char *value)
{
	char topic[1024];
	strcpy(topic, "state");
	strncat(topic, type, sizeof(topic));
	publish(NULL, topic, strlen(value), value);
}

/* Receive car control messages from server and store them in corresponding values locally.
   The values are then forwared to SD2. */
void savm::on_message(const struct mosquitto_message *message)
{
	//Genode::log("%s %s", message->topic, message->payload);

	starttime = timer.elapsed_ms();

	/* Take first part of message, alias identifier to decide which value arrived */

	char *type = strrchr(message->topic, '/') + 1;
	char *value = (char *)message->payload;

	//std::string payload = (char *)message->payload;
	//const char *name = payload.substr(0, payload.find(",")).c_str();

	if (!strcmp(type, "0"))
	{
		steer = atof(value);
	}
	if (!strcmp(type, "1"))
	{
		brake = atof(value);
	}
	if (!strcmp(type, "2"))
	{
		steer = atof(value);
	}
	if (!strcmp(type, "3"))
	{
		float tmp = atof(value);
		if (tmp > 0)
		{
			autonomous = true;
		}
		else
		{
			autonomous = false;
		}
	}
	if (!strcmp(type, "4"))
	{

		speed = atof(value);
	}

	sem_wait(&allValSem);
	allValues = (allValues + 1) % 4;
	if (!allValues)
	{
		sem_post(&allData);
	}
	sem_post(&allValSem);

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

void savm::on_connect(int rc)
{
	Genode::log("connected to mosquitto server");
}

void savm::on_disconnect(int rc)
{
	Genode::log("disconnect from mosquitto server");
}

savm::savm(const char *id, Libc::Env &_env) : mosquittopp(id)
{
	/* initialization */
	sem_init(&allValSem, 0, 1);
	sem_init(&allData, 0, 0);
	mosqpp::lib_init();
	Timer::Connection _timer;
	//lwip_tcpip_init(); /* causes freeze, code works fine without it */

	Genode::Attached_rom_dataspace _config(_env, "config");
	/* get config */
	Genode::Xml_node mosquitto = _config.xml().sub_node("mosquitto");

	/***********************
	 ** Connection to SD2 **
	 ***********************/
	int sock;

	char ip_addr[16] = {0};
	unsigned int port = 0;

	int ret;
	Genode::Xml_node speeddreams = _config.xml().sub_node("speed-dreams");
	try
	{
		speeddreams.attribute("ip-address").value(ip_addr, sizeof(ip_addr));
		speeddreams.attribute("port").value<unsigned int>(&port);
	}
	catch (Genode::Xml_node::Nonexistent_attribute)
	{
		Genode::error("please check speed-dreams configuration");
	}

	struct sockaddr_in srv_addr;
	bzero(&srv_addr, sizeof(srv_addr));
	srv_addr.sin_family = AF_INET;
	srv_addr.sin_addr.s_addr = inet_addr(ip_addr);
	srv_addr.sin_port = htons(port);

	if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		Genode::error("socket failed: ", (const char *)strerror(errno));
	}

	while ((ret = lwip_connect(sock, (struct sockaddr *)&srv_addr, sizeof(srv_addr)) == -1))
	{
		Genode::error("connect failed: ", (const char *)strerror(errno));
		lwip_close(sock);
		if ((sock = lwip_socket(AF_INET, SOCK_STREAM, 0)) == -1)
		{
			Genode::error("socket failed: ", (const char *)strerror(errno));
		}
	}

	/* disable Nagle's algorithm */
	int flag = 1;
	int result = lwip_setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
	if (result == -1)
	{
		Genode::error("setsockopt failed: ", (const char *)strerror(errno));
	}

	try
	{
		mosquitto.attribute("ip-address").value(this->host, sizeof(host));
	}
	catch (Genode::Xml_node::Nonexistent_attribute)
	{
		Genode::error("mosquitto ip-address is missing from config");
	}
	this->port = mosquitto.attribute_value<unsigned int>("port", 1883);
	this->keepalive = mosquitto.attribute_value<unsigned int>("keepalive", 60);

	/* connect to mosquitto server */
	do
	{
		ret = this->connect(host, port, keepalive);
		switch (ret)
		{
		case MOSQ_ERR_INVAL:
			Genode::error("invalid parameter for mosquitto connect");
			return;
		case MOSQ_ERR_ERRNO:
			break;
			Genode::log("mosquitto ", (const char *)strerror(errno));
		}
	} while (ret != MOSQ_ERR_SUCCESS);

	/* subscribe to topic */
	do
	{
		ret = this->subscribe(NULL, topic);
		switch (ret)
		{
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
	} while (ret != MOSQ_ERR_SUCCESS);

	Genode::log("for loop mosq");
	/* start non-blocking mosquitto loop */
	loop_start();

	char val[512] = {0};
	uint32_t msg_len = 0;
	std::string cdi_str;
	char buffer[1024] = {0};
	/* Init size of protobuf message with 0 */
	int size = 0;
	/* Allocate ram dataspace where protobuf message will be stored */
	Genode::Ram_dataspace_capability state_ds = _env.ram->alloc(1024);
	/* Attach dataspace to our rm session */
	char *bar = _env.rm().attach(state_ds);
	while (true)
	{
		CONNstarttime = timer.elapsed_ms();

		/*
		readAllBytes(&msg_len, sock, sizeof(msg_len));
		msg_len = ntohl(msg_len);

		buffer[msg_len] = {'\0'};
		readAllBytes(buffer, sock, msg_len);
		*/

		size = 0;
		/* Get size of message from SD2 */
		lwip_read(sock, &size, ntohl(4));

		/*******************
		 ** SensorDataOut **
		 *******************/
		
		if (size > 0)
		{
			
			lwip_read(sock, bar, ntohl(size));
			protobuf::State state;
			/* Deserialize from char* to state */
			state.ParseFromArray(bar, size);
			/* Publish any value we got from SD2 */
			snprintf(val, sizeof(val), "%f", state.steer());
			myPublish("steer", val);
			snprintf(val, sizeof(val), "%f", state.brakecmd());
			myPublish("brake", val);
			snprintf(val, sizeof(val), "%f", state.accelcmd());
			myPublish("accel", val);
			snprintf(val, sizeof(val), "%f", state.wheel(0).spinvel());
			myPublish("wheel0", val);
			snprintf(val, sizeof(val), "%f", state.wheel(1).spinvel());
			myPublish("wheel1", val);
			snprintf(val, sizeof(val), "%f", state.wheel(2).spinvel());
			myPublish("wheel2", val);
			snprintf(val, sizeof(val), "%f", state.wheel(3).spinvel());
			myPublish("wheel3", val);
			snprintf(val, sizeof(val), "%f", state.specification().length());
			myPublish("length", val);
			snprintf(val, sizeof(val), "%f", state.specification().width());
			myPublish("width", val);
			snprintf(val, sizeof(val), "%f", state.specification().wheelradius());
			myPublish("wheelRadius", val);
			snprintf(val, sizeof(val), "%f", state.specification().steerlock());
			myPublish("steerlock", val);
			snprintf(val, sizeof(val), "%f", state.timestamp());
			myPublish("timestamp", val);
			/* Go through sensors */
			for (int i = 0; i < state.sensor().size(); i++)
			{
				/* GPS sensor */
				if (state.sensor(i).type() == protobuf::Sensor_SensorType_GPS)
				{
					snprintf(val, sizeof(val), "%f", state.sensor(i).value(0));
					myPublish("gps_x", val);
					snprintf(val, sizeof(val), "%f", state.sensor(i).value(1));
					myPublish("gps_y", val);
				}
				/* Laser sensor */
				else
				{
					snprintf(val, sizeof(val), "%f", state.sensor(i).value(0));
					char buffer[1024] = {0};
					sprintf(buffer, "laser%d", i);
					const char *name = buffer;
					myPublish(name, val);
				}
			}

			sem_wait(&allData);
			/* String control data is serialized to */
			std::string foo;
			protobuf::Control ctrl;
			/* Set values received from ECU/Parking to control */
			ctrl.set_steer(steer);
			ctrl.set_accelcmd(accel);
			ctrl.set_brakecmd(brake);
			//Genode::log("Bremse ist", brake);
			ctrl.set_speed(speed);
			ctrl.set_autonomous(autonomous);
			/* Serialize control to String */
			ctrl.SerializeToString(&foo);
			/* Get size of serialized String */
			int32_t m_length = foo.size();
			msg_len = m_length;
			/* Send size of serialized String to SD2 */
			//lwip_write(_listen_socket, &m_length, 4);
			//* Send serialized String to SD2 */
			//lwip_write(_listen_socket, (void *)foo.c_str(), foo.size());
			/* send message */

			int ret = lwip_write(sock, &m_length, 4);
			if (ret == -1)
			{
				Genode::error("write cdi failed! ", (const char *)strerror(errno));
			}
			else if ((unsigned int)ret < msg_len)
			{
				Genode::error("write cdi failed to send complete message! ",
							  ret,
							  " vs. ",
							  msg_len);
			}

			ret = lwip_write(sock, (void *)foo.c_str(), foo.size());
			if (ret == -1)
			{
				Genode::error("write cdi failed! ", (const char *)strerror(errno));
			}
			else if ((unsigned int)ret < msg_len)
			{
				Genode::error("write cdi failed to send complete message! ",
							  ret,
							  " vs. ",
							  msg_len);
			}
		}
		else
		{
			Genode::error("Unknown message: %d", msg_len);
		}
		_env.ram->free(state_ds);
	}

	CONNstoptime = timer.elapsed_ms();
	CONNduration = CONNstoptime - CONNstarttime;
	CONNtotalduration += CONNduration;
	CONNcalculationroundscounter++;

	if (CONNcalculationroundscounter == 1)
	{
		CONNminval = CONNduration;
	}
	CONNminval = std::min(CONNminval, CONNduration);
	CONNmaxval = std::max(CONNmaxval, CONNduration);
	if (CONNcalculationroundscounter % 500 == 0)
	{
		Genode::log("The CONNduration for calculation and sending was ", CONNduration, " milliseconds");
		Genode::log("The CONNTOTALduration for calculation and sending was ", CONNtotalduration, " milliseconds");
		Genode::log("The CONNMINduration for calculation and sending was ", CONNminval, " milliseconds");
		Genode::log("The CONNMAXduration for calculation and sending was ", CONNmaxval, " milliseconds");
		Genode::log("The CONNAVERAGEduration for calculation and sending was ", (CONNtotalduration / CONNcalculationroundscounter), " milliseconds after ", CONNcalculationroundscounter, " steps");
	}
	Genode::env()->pd_session.free(state_ds);
}

/* TODO */
savm::~savm()
{
}

/* cleanup */
//	mosqpp::lib_cleanup();
//}
