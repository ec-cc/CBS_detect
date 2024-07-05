#pragma once
#include<iostream>
#include<winsock.h>
#include<vector>
#include<queue>
#include "json.h"
#include"extern.h"

#pragma comment(lib,"ws2_32.lib")
const int len_double = sizeof(double);
void initialization();
/***********jsonת��*************/
namespace nlohmann {
	namespace js_to_data
	{
		struct sendData
		{
			int ID;
			int flag;
			std::vector<Point> data;
		};
		void to_json(json &j, const Point &r);
		void from_json(const json&j, Point &r);
		void to_json(json &j, const sendData &r);
		void from_json(const json &j, sendData &r);
	}
}
void TCP_TRANS(int port, const char* IP, std::vector<nlohmann::js_to_data::sendData>result);

void TCP_Connect(int port, const char* IP);                                  //��������������
void TCP_Send(std::vector<nlohmann::js_to_data::sendData>sendD);                            //����
void TCP_Receive(std::vector<nlohmann::js_to_data::sendData>&recData);                       //����
void TCP_ShutDown();                                                         //�ر�
void TCP_ReceiveStr(std::string &resultStr);
void TCP_SendStr(std::string str);


void TCP_Connect(int port, const char* IP, SOCKET &sClient, SOCKADDR_IN &clientAddr);           //��������������
void TCP_Send(std::vector<nlohmann::js_to_data::sendData>sendD, SOCKET sClient);                             //����
void TCP_Receive(std::vector<nlohmann::js_to_data::sendData>&recData, SOCKET sClient);                       //����
void TCP_ShutDown(SOCKET sClient);                                                           //�ر�
void TCP_ReceiveStr(std::string &resultStr, SOCKET sClient);                                      //���ַ���������֤Ч��
void TCP_SendStr(std::string str, SOCKET sClient);

////////////////////////////
//ȫ�ֱ���
extern SOCKET s_client1;								//���������׽���
extern SOCKADDR_IN client_addr1;
extern SOCKET s_client2;								//���������׽���
extern SOCKADDR_IN client_addr2;
extern SOCKET s_client3;								//���������׽���
extern SOCKADDR_IN client_addr3;
extern SOCKET s_client4;								//���������׽���
extern SOCKADDR_IN client_addr4;
extern SOCKET s_client5;								//���������׽���
extern SOCKADDR_IN client_addr5;
////////////////////////////
//���ݴ�����
//bool proMySend(vector<js_to_data::sendData>&mySend, vector<queue<Point>>&myResult);
