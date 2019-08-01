#pragma once

#include <stdio.h>
#include <Windows.h>
#include <iostream>

#pragma comment(lib,"ws2_32.lib")
#define  PORT 5000
#define BUFFER_SIZE 1024
// #define  IP_ADDRESS "192.168.143.102"

class RSocket
{
public:
	RSocket();
	~RSocket();

private:
	
public:
	SOCKET InitSocket(int Sport, const char* ip);
	void Rsend(SOCKET s, const char *SendBuffer);
	void Accept(SOCKET s, char* &SendBuffer);
	void close(SOCKET ClientSocket);
};

