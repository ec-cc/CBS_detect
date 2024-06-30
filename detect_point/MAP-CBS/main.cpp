//#include "CBSDataStructures.h"
#include <iostream>
#include <ctime>
#include <exception>
#include "dataTrans.h"
#include "CBS.h"
using namespace std;

extern const double collideDis = 1500;		//碰撞距离

//*************************************************************
////////多智能体
//*************************************************************

//TODO general list
//--throw exception when input cannot be read   

//++ cells with obstacle
//++ solution for the conflict(needs to be tested)
//++ cost, get_best_node
//++ read from input file
//++ input with different size grid

//-Note that for a given CT node N, one does not have to save all its cumulative constraints.
//Instead, it can save only its latest constraint and extract the other constraints by traversing the path from N to the root via its ancestors.
//Similarly, with the exception of the root node, the low - level search should only be performed for agent ai which is associated with the 
//newly added constraint.The paths of other agents remain the same as no new constraints are added for them.


int main()
{
	HighLevelCBS CBS;
	clock_t on, off;     

	//CBS.addAgent(0, 0, 12000, 130, 200, 20000, -37000, 180, 0);
	//CBS.addAgent(1, 40000, 10000, 130, 270, 20000, -37000, 135, 0);
	//CBS.addAgent(2, -10000, 50000, 130, 200, 20000, -37000, 90, 0);
	//CBS.addAgent(3, -10000, 18000, 130, 270, 20000, -37000, 120, 0);
	
	CBS.addAgent(0, -10000, 12000, 130, 200, 20000, -37000, 0, 0);
	CBS.addAgent(1, -10000, 10000, 130, 270, 20000, -37000, 0, 0);
	CBS.addAgent(2, -10000, 14000, 130, 200, 20000, -37000, 0, 0);
	CBS.addAgent(3, -10000, 18000, 130, 270, 20000, -37000, 0, 0);

	//CBS.addAgent(0, -43245, 13224,      104.557, 166.36, -7719.51, -1321.04, 0, 0);
	//CBS.addAgent(1, -37248.8, -22325.2, 74.803, 208.306, -7719.51, -1321.04, 0, 0);
	//CBS.addAgent(2, -39295, -7691,      82.307, 173.386, -7719.51, -1321.04, 0, 0);
	//CBS.addAgent(3, -31709.5, 7950.48,  146.529, 166.204,-7719.51, -1321.04, 0, 0);

	//CBS.addAgent(0, -38219.7, 22976.5,  101.346, 166.307,-7719.51, -1321.04, 0, 0);
	//CBS.addAgent(1, -38030.2, -22491.2, 78.714, 194.713, -7719.51, -1321.04, 0, 0);
	//CBS.addAgent(2, -30675.1, -13794.5, 113.84, 207.228, -7719.51, -1321.04, 0, 0);
	//CBS.addAgent(3, -32110.2, 8476.2,   141.516, 166.329, -7719.51, -1321.04, 0, 0);
	
	//CBS.addAgent(0, -37294.4, 20306.2,  69.28,   166.246, 2290.33, -6681.66, 0, 0);
	//CBS.addAgent(1, -36676.5, -19791.5, 108.442, 208.108, 2290.33, -6681.66, 0, 0);
	//CBS.addAgent(2, -28243.6, -15991.7, 112.812, 272.148, 2290.33, -6681.66, 0, 0);
	//CBS.addAgent(3, -34135.6, 89.3453,  129.804, 166.2,   2290.33, -6681.66, 0, 0);

	//no  ---该组数据无解 会出bug
	//CBS.addAgent(0, -34370.6, 21404.2,   69.4709, 166.265, 5542.69, -12722.1, 0, 0);
	//CBS.addAgent(1, -33030.4, -21017.6,  108.255, 205.047, 5542.69, -12722.1, 0, 0);
	//CBS.addAgent(2, -23467,   -17832.6,  108.775, 272.214, 5542.69, -12722.1, 0, 0);
	//CBS.addAgent(3, -31742,   -19298.82, 129.939, 166.218, 5542.69, -12722.1, 0, 0);

	CBS.InitialCBS();

	on = clock();
	auto solution = CBS.RunCBS();
	
	off = clock();
	std::cout <<"time: "<< (double)(off - on) / CLOCKS_PER_SEC << endl; 

	auto dtp = CBS.resCTN->get_dtp();
	for (int j = 0; j < dtp.size(); j++)
	{
		cout << "UAV" << dtp[j].agentIndex << ":  " << "   " << dtp[j].tStart << "   " << dtp[j].tEnd << endl;
	}

	//sort(dtp.begin(), dtp.end(), [](const detectTimePeriod& a, const detectTimePeriod& b) {
	//	if (isEqualDouble(a.tStart, b.tStart)) {
	//		return a.tEnd < b.tEnd;
	//	}
	//	else {
	//		return a.tStart < b.tStart;
	//	}
	//});
	//getDTPSolution(dtp, solution);


	//-----------------------------------------------------------------------------
	auto kk = solution.size();
	if (!kk)
	{
		cout << "没有结果！！！！" << endl;
		system("pause");
		return 0;
	}
	TCP_Connect(1, "127.0.0.1", s_client1, client_addr1);
	
	string time = to_string(solution.size());
	TCP_SendStr(time,s_client1);

	string xxx, yyy;
	for (int k = 0; k < solution.size(); k++)
	{
		xxx = "";
		yyy = "";
		for (int i = 0; i < solution[k]->Nodes.size() - 1; i++)
		{
			xxx = xxx + to_string(solution[k]->Nodes[i]->x);
			xxx += ' ';
			yyy = yyy + to_string(solution[k]->Nodes[i]->y);
			yyy += ' ';

		}
		Sleep(400);
		TCP_SendStr(xxx, s_client1);
		Sleep(400);
		TCP_SendStr(yyy, s_client1);		
	}
	Sleep(400);
	string endx, endy;
	endx = to_string(solution[0]->Nodes[solution[0]->Nodes.size() - 1]->x);
	endy = to_string(solution[0]->Nodes[solution[0]->Nodes.size() - 1]->y);
	string endP = endx;
	endP += ' ';
	endP += endy;
	endP += ' ';
	TCP_SendStr(endP, s_client1);
	//TCP_SendStr(endy, s_client1);

	string t;
	for (int h = 0; h < dtp.size(); h++)
	{
		t = "";
		t = t + to_string(dtp[h].tStart);
		t += ' ';
		t = t + to_string(dtp[h].tEnd);
		t += ' ';
		Sleep(400);
		TCP_SendStr(t, s_client1);
	}



	//CBS.clearCBS();   //agents 
	system("pause");

	return 0;
}