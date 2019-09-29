#include <mosquittopp.h>
#include <semaphore.h>
#include <base/component.h>
#include <libc/component.h>
/* protobuf include */
#include <state.pb.h>
#include <control.pb.h>

/* fix redefinition of struct timeval */
#define timeval _timeval

class savm : public mosqpp::mosquittopp
{
	private:
	/* mosquitto */
	char host[16];
	const char* id = "savm";
	const char* topic = "car-control/#";
	const char* topicsub = "state/#";
	int port;
	int keepalive;

	int allValues;
	sem_t allValSem;
	sem_t allData;

	void on_connect(int rc);
	void on_disconnect(int rc);
	void on_message(const struct mosquitto_message *message);

	void readAllBytes(void *buf, int socket, unsigned int size);
	void myPublish(const char *type, float value);

	public:
	savm(const char* id, Libc::Env &env);
	~savm();
};
