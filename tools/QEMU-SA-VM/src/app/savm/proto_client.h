#pragma once

extern "C" {
#include <lwip/sockets.h>
}

#include <base/attached_rom_dataspace.h>
#include <libc/component.h>

class Proto_client
{
public:
	Proto_client(Genode::Attached_rom_dataspace &_config);

	~Proto_client();

	int connect();

	void serve(Publisher *publisher, Genode::Env &_env);

	void disconnect();

private:
	int _listen_socket;
	struct sockaddr_in _in_addr;
	sockaddr _target_addr;
};
