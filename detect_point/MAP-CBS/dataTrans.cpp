#include "dataTrans.h"

using namespace std;
using namespace nlohmann;
using js_to_data::sendData;

/********************ȫ�ֱ���************************/
//���峤�ȱ���
int send_len = 0;
int recv_len = 0;
//���巢�ͻ������ͽ��ܻ�����
const int BUFF_SIZE = 10000;
char send_buf[BUFF_SIZE];
char recv_buf[BUFF_SIZE];
//���������׽��֣����������׽���
SOCKET s_client;
SOCKET s_client1;
SOCKET s_client2;
SOCKET s_client3;
SOCKET s_client4;
SOCKET s_client5;

//����˵�ַ�ͻ��˵�ַ
SOCKADDR_IN client_addr;
SOCKADDR_IN client_addr1;
SOCKADDR_IN client_addr2;
SOCKADDR_IN client_addr3;
SOCKADDR_IN client_addr4;
SOCKADDR_IN client_addr5;

int index = 0;
/*********************************************************/
void initialization() {
	//��ʼ���׽��ֿ�
	WORD w_req = MAKEWORD(2, 2);//�汾��
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "��ʼ���׽��ֿ�ʧ�ܣ�" << endl;
	}
	else {
		cout << "��ʼ���׽��ֿ�ɹ���" << endl;
	}
	//���汾��
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "�׽��ֿ�汾�Ų�����" << endl;
		WSACleanup();
	}
	else {
		cout << "�׽��ֿ�汾��ȷ��" << endl;
	}
	//������˵�ַ��Ϣ

}

void TCP_Connect(int port, const char* IP)
{
	char  recvline[4096], sendline[1000];
	double joint[2][6] = { 10.123456,11.123456,12.123456,13.123456,14.123456,15.123456,16.123456,17.123456,18.123456,19.123456,20.123456,21.123456 };
	double joint1[7] = { 0 };
	//portΪ�˿ں�,ipΪ��ַ ������ַΪ "127.0.0.1"
	/***************����ת������****************************/
	if (index == 0)
	{
		//��ʼ���׶�
		initialization();
		client_addr.sin_family = AF_INET;
		client_addr.sin_addr.S_un.S_addr = inet_addr(IP);
		client_addr.sin_port = htons(port);
		//�����׽���
		s_client = socket(AF_INET, SOCK_STREAM, 0);
		if (connect(s_client, (SOCKADDR *)&client_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
			cout << "����������ʧ�ܣ�" << endl;
			WSACleanup();
		}
		else {
			cout << "���������ӳɹ���" << endl;
		}
		/*index = 1;*/
	}
}

void TCP_Send(vector<js_to_data::sendData>sendD)
{
	json data;
	for (js_to_data::sendData i : sendD)
	{
		json j;
		js_to_data::to_json(j, i);
		data.emplace_back(j);
	}
	send_len = send(s_client, &data.dump()[0], data.dump().size() * sizeof(char), 0);
	if (send_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else
	{
		//sprintf("")
	}
}

void TCP_SendStr(string str)
{
	int len = str.length() + 1;
	strcpy_s(send_buf, str.c_str());
	send_len = send(s_client, send_buf, len, 0);
	if (send_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else
	{
		//sprintf("")
	}
}

void TCP_ReceiveStr(string &resultStr)
{
	/******************��������****************************/
	recv_len = recv(s_client, recv_buf, 1000, 0);
	recv_buf[recv_len] = '\0';
	if (recv_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else {
		resultStr = recv_buf;
	}
}
void TCP_Receive(vector<js_to_data::sendData>&recData)
{
	/******************��������****************************/
	recv_len = recv(s_client, recv_buf, 1000, 0);
	recv_buf[recv_len] = '\0';
	if (recv_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else {
		//��Ҫ�ѽ��յ����ݽ�������ת��
		json temp = json::parse(string(recv_buf));
		auto r = temp.get<vector<js_to_data::sendData>>();
		recData = r;
	}
}
void TCP_ShutDown()
{
	//�ر��׽���
	closesocket(s_client);
	//�ͷ�DLL��Դ
	WSACleanup();
}

void TCP_Connect(int port, const char* IP, SOCKET &sClient, SOCKADDR_IN &clientAddr)
{
	char  recvline[4096], sendline[2000];
	//portΪ�˿ں�,ipΪ��ַ ������ַΪ "127.0.0.1"
	/***************����ת������****************************/
	/*if (index == 0)
	{*/
	//��ʼ���׶�
	initialization();
	clientAddr.sin_family = AF_INET;
	clientAddr.sin_addr.S_un.S_addr = inet_addr(IP);
	clientAddr.sin_port = htons(port);
	//�����׽���
	sClient = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(sClient, (SOCKADDR *)&clientAddr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "����������ʧ�ܣ�" << endl;
		WSACleanup();
	}
	else {
		cout << "���������ӳɹ���" << endl;
	}
	/*}*/
}
void TCP_Send(vector<js_to_data::sendData>sendD, SOCKET sClient)
{
	json data;
	for (js_to_data::sendData i : sendD)
	{
		json j;
		js_to_data::to_json(j, i);
		data.emplace_back(j);
	}
	send_len = send(sClient, &data.dump()[0], data.dump().size() * sizeof(char), 0);
	if (send_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else
	{
		//sprintf("")
	}
}
void TCP_SendStr(string str, SOCKET sClient)
{
	int len = str.length() + 1;
	strcpy_s(send_buf, str.c_str());
	send_len = send(sClient, send_buf, len, 0);
	if (send_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else
	{
		//sprintf("")
	}
}

void TCP_Receive(vector<js_to_data::sendData>&recData, SOCKET sClient)
{
	/******************��������****************************/
	recv_len = 0;
	while (recv_len <= 0)
	{
		recv_len = recv(sClient, recv_buf, 1000, 0);

	}
	recv_buf[recv_len] = '\0';
	if (recv_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else {
		//��Ҫ�ѽ��յ����ݽ�������ת��
		json temp = json::parse(string(recv_buf));
		auto r = temp.get<vector<js_to_data::sendData>>();
		recData = r;
	}
}
void TCP_ReceiveStr(string &resultStr, SOCKET sClient)
{
	/******************��������****************************/
	recv_len = 0;
	while (recv_len <= 0)
	{
		recv_len = recv(sClient, recv_buf, 1000, 0);

	}
	recv_buf[recv_len] = '\0';
	if (recv_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else {
		resultStr = recv_buf;
	}
}
void TCP_ShutDown(SOCKET sClient)
{
	//�ر��׽���
	closesocket(sClient);
	//�ͷ�DLL��Դ
	WSACleanup();
}







void TCP_TRANS(int port, const char* IP, vector<js_to_data::sendData>result)
{
	char  recvline[4096], sendline[1000];
	double joint[2][6] = { 10.123456,11.123456,12.123456,13.123456,14.123456,15.123456,16.123456,17.123456,18.123456,19.123456,20.123456,21.123456 };
	double joint1[7] = { 0 };
	//portΪ�˿ں�,ipΪ��ַ ������ַΪ "127.0.0.1"
	/***************����ת������****************************/
	if (index == 0)
	{
		//��ʼ���׶�
		initialization();
		client_addr.sin_family = AF_INET;
		client_addr.sin_addr.S_un.S_addr = inet_addr(IP);
		client_addr.sin_port = htons(port);
		//�����׽���
		s_client = socket(AF_INET, SOCK_STREAM, 0);
		if (connect(s_client, (SOCKADDR *)&client_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
			cout << "����������ʧ�ܣ�" << endl;
			WSACleanup();
		}
		else {
			cout << "���������ӳɹ���" << endl;
		}
	}
	json data;

	for (js_to_data::sendData i : result)
	{
		json j;
		js_to_data::to_json(j, i);
		data.emplace_back(j);
	}
	send_len = send(s_client, &data.dump()[0], data.dump().size() * sizeof(char), 0);
	if (send_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else
	{
		//sprintf("")
	}
	/******************��������****************************/
	recv_len = recv(s_client, recv_buf, 1000, 0);
	recv_buf[recv_len] = '\0';
	if (recv_len < 0) {
		cout << "����ʧ�ܣ�" << endl;
	}
	else {
		//��Ҫ�ѽ��յ����ݽ�������ת��
		string sss = recv_buf;
		cout << sss << endl;
		json temp = json::parse(string(recv_buf));
		auto r = temp.get<vector<js_to_data::sendData>>();
	}
	//�ر��׽���
	closesocket(s_client);
	//�ͷ�DLL��Դ
	WSACleanup();
}
/*************************************json��ṹ��ת��***************************************************/
void js_to_data::to_json(json &j, const Point &r)
{
	j = json{ {"x",r.x},{"y",r.y},{"z",r.z},{"azimuth",r.azimuth},{"v",r.v} };
}
void js_to_data::from_json(const json&j, Point &r)
{
	j.at("x").get_to(r.x);
	j.at("y").get_to(r.y);
	j.at("z").get_to(r.z);
	j.at("azimuth").get_to(r.azimuth);
	j.at("v").get_to(r.v);
}
void js_to_data::to_json(json &j, const sendData &r)
{
	j = json{ {"ID",r.ID}, {"flag",r.flag} };
	for (auto i : r.data)
	{
		json ia;
		to_json(ia, i);
		j["data"].emplace_back(ia);
	}
}
void js_to_data::from_json(const json &j, sendData &r)
{
	auto a = j["ID"];
	j.at("ID").get_to(r.ID);
	j.at("flag").get_to(r.flag);
	for (auto const& i : j.at("data"))
	{
		Point a;
		from_json(i, a);
		r.data.emplace_back(move(a));
	}
}
/**************************************************************/
//���������ݴ�����
bool proMySend(vector<sendData>&mySend, vector<queue<Point>>&myResult)
{

	for (int i = 0; i < myResult.size(); i++)  //�з����ҷ�
	{
		switch (i)                           //�ⲿ��Ҫ��
		{
		case 0:
			mySend[i].ID = 14111;
			mySend[i].flag = 0;
			break;
		case 1:
			mySend[i].ID = 14112;
			mySend[i].flag = 0;
			break;
		case 2:
			mySend[i].ID = 20111;
			mySend[i].flag = 1;
			break;
		default:
			break;
		}
		int temp = myResult[i].size();

		for (int j = 0; j < temp; j++)
		{
			mySend[i].data.push_back(myResult[i].front());
			myResult[i].pop();
		}
		reverse(mySend[i].data.begin(), mySend[i].data.end());
	}
	return true;
}