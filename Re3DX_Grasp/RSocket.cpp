#include "RSocket.h"


RSocket::RSocket()
{
	//ClientSocket = INVALID_SOCKET;
}


RSocket::~RSocket()
{
}

SOCKET RSocket::InitSocket(int S_port, const char* ip){
	WSADATA  Ws;
	SOCKET ClientSocket;
	struct sockaddr_in ClientAddr;
	int Ret = 0;
	/* Init Windows Socket */
	if (WSAStartup(MAKEWORD(2, 2), &Ws) != 0)
	{
		printf("Init Windows Socket Failed::%d\n", GetLastError());
		return -1;
	}

	/* Create Socket */
	ClientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (ClientSocket == INVALID_SOCKET)
	{
		printf("Create Socket Failed::%d\n", GetLastError());
		return -1;
	}

	ClientAddr.sin_family = AF_INET;
	ClientAddr.sin_addr.s_addr = inet_addr(ip);
	ClientAddr.sin_port = htons(S_port);
	memset(ClientAddr.sin_zero, 0x00, 8);

	/* connect socket */
	Ret = connect(ClientSocket, (struct sockaddr*)&ClientAddr, sizeof(ClientAddr));
	if (Ret == SOCKET_ERROR)
	{
		printf("Connect Error::%d\n", GetLastError());
		return -1;
	}
	else
	{
		printf("Connect succedded!\n");
	}

	return ClientSocket;
}

void RSocket::Rsend(SOCKET s, const char *SendBuffer){
	/* send data to server */
	if (send(s, SendBuffer, (int)strlen(SendBuffer), 0) == SOCKET_ERROR)
	{
		printf("Send Info Error::%d\n", GetLastError());
	}
}

void RSocket::Accept(SOCKET s, char* &SendBuffer) {
	if(recv(s, SendBuffer, BUFFER_SIZE, 0) == SOCKET_ERROR)
		printf("recv Info Error::%d\n", GetLastError());
}

void RSocket::close(SOCKET ClientSocket){
	/* close socket */
	Rsend(ClientSocket, "q");
	closesocket(ClientSocket);
	WSACleanup();
}
