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
/***********json转换*************/
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

void TCP_Connect(int port, const char* IP);                                  //服务器建立连接
void TCP_Send(std::vector<nlohmann::js_to_data::sendData>sendD);                            //发送
void TCP_Receive(std::vector<nlohmann::js_to_data::sendData>&recData);                       //接收
void TCP_ShutDown();                                                         //关闭
void TCP_ReceiveStr(std::string &resultStr);
void TCP_SendStr(std::string str);


void TCP_Connect(int port, const char* IP, SOCKET &sClient, SOCKADDR_IN &clientAddr);           //服务器建立连接
void TCP_Send(std::vector<nlohmann::js_to_data::sendData>sendD, SOCKET sClient);                             //发送
void TCP_Receive(std::vector<nlohmann::js_to_data::sendData>&recData, SOCKET sClient);                       //接收
void TCP_ShutDown(SOCKET sClient);                                                           //关闭
void TCP_ReceiveStr(std::string &resultStr, SOCKET sClient);                                      //用字符串接受验证效果
void TCP_SendStr(std::string str, SOCKET sClient);

////////////////////////////
//全局变量
extern SOCKET s_client1;								//定义服务端套接字
extern SOCKADDR_IN client_addr1;
extern SOCKET s_client2;								//定义服务端套接字
extern SOCKADDR_IN client_addr2;
extern SOCKET s_client3;								//定义服务端套接字
extern SOCKADDR_IN client_addr3;
extern SOCKET s_client4;								//定义服务端套接字
extern SOCKADDR_IN client_addr4;
extern SOCKET s_client5;								//定义服务端套接字
extern SOCKADDR_IN client_addr5;
////////////////////////////
//数据处理函数
//bool proMySend(vector<js_to_data::sendData>&mySend, vector<queue<Point>>&myResult);
