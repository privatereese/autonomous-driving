#include <mosquittopp.h>
#include <semaphore.h>
#include <base/component.h>
#include <libc/component.h>
#include <stdlib.h>

/* fix redefinition of struct timeval */
#define timeval _timeval

class ecu : public mosqpp::mosquittopp
{
private:
	/* mosquitto */
	char host[16];
	const char *id = "ecu";
	const char *topic = "car-control/#";
	const char *topicsub = "state/#";
	int port;
	int keepalive;

	int allValues;
	sem_t allValSem;
	sem_t allData;

	void on_connect(int rc);
	void on_disconnect(int rc);
	void on_message(const struct mosquitto_message *message);

	void readAllBytes(void *buf, int socket, unsigned int size);
	

public:
	ecu(const char *id, Libc::Env &env);
	void myPublish(const char *type, float value);
	~ecu();
};
