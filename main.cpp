#define _CRT_SECURE_NO_WARNINGS
/*
* Design by Wang.jh.
* Based on "demo.cpp" by Huang.wj
*
* ��������������
*
* 1.�������ṹ��������
*	�������Ա��������
*	  robot_assign()-���ݻ�����״̬ ��������/�Ƿ�Я��������ݲ�ͬ��״̬Ϊ���л����˷��ͻ����λ��Ŀ��λ��
*	  output_move()-��output_move������л����˵�moveָ��
*     output_get()-������л����˵�getָ������˵��ж�����ǲ�get��������û����˵�getָ��
*	  output_pull()-ͬ��

* 2.�����˽ṹ��������
*	�������������ĳ�Ա������
*	  task��0-������ 1-������Ѱ�һ���/Ѱ�Ҳ�λ��	ִ��pull�Ժ�Ὣtask����,����������task=0�Ļ�����ʱ���ڷ���Ŀ���ͬʱ��task��1
*   ��Ա������
*	  int movenumber():������������Ŀ��������ͬʱ�������������񶼲��ܶ�����Ŀ��λ�ò�ͬʱ����task������A*��Ŀ��ǰ��
*	  int getnumber():����������£�û��Я������ �� ����������Ŀ�������غ�ִ��get
*	  int pullnumber():Я���������Ŀ���غ�ʱִ��pull
*
* 3.����˵����
* �����ض�������Ʒ������㷨δ���
* Ѱַ�㷨δ���
* �����˵��߼�ֻ����һ֡���һ���������ŵ����� �ȶ���ȱ��ʱ�������һ֡��ͬʱִ��move��get�Ĺ��̻���һ֡
* �޸���Robot�Ĺ��캯������Ϊȱʡ�ͱ��ڳ�ʼ��������Ա����
* bug������ϵ�Ŀ��λ��ǰ������ʧ�ˣ��������һֱget��ѭ��
*
*
* ��ѡ��������ȡ�洢��·��(map)�����ƶ������µĻ�������Ϊ�߼���
* 4.�����˶�ȡ�洢��·��(map)�����ƶ���˼·��������Ҫһ������i��¼map�ϵ�·���Ѿ����˼�����������ִ�е�i+1��
*							���Կ��Ƕ���ṹ��map����Ա��������Ŀ��λ�ã��滮��·������¼�ߵ��ڼ����ĳ�Ա����i��
*							�Լ���¼�Ƿ��Ѿ�ִ��getָ��ĳ�Ա����get_already
* get��pull���жϼ�һ����������map·�����ꡱ��������map��ômove�ǲ����ģ����·�߹滮��/������ʧ�����������get���
*							�û����˲�����뵽Я�������״̬���������ǲ����·�����λ��map()����������Զ����������
*							Ϊ��������⣬�����û���������map��getһ�Σ�����get_already=1��
*							��������Ϊget_already==1&&goods==0�Ļ������·�һ���µĵ�ͼ��Ѱ���µ�Ŀ�꣬pull����
* move�������Ŀ�����겻ͬ:1.����ж��� ��û����map�������map��()��
*
*							���������map�ϵ�·������������»����������
*							��Ŀ�������غϣ��ν���get��pull���жϣ�
*
*						   2.����Ѿ������ͼ�ϵ�·����˵���滮·�������Ȳ������ȴ����������յ�·�ߴ����ָ��(��get�����жϷ���)��
*								�����µ�map
*		�����Ŀ��������ͬ������
*
* ���䣺����滮��һ��·�ߺ�ִ�е��ף���һ��Ŀ������ѡ�����
*/

//    3��17��
/////�޸ĺ���ȫ��ʵ��������Robot�еĳ�Ա����MAP�ṹ��ɾ����ֻʹ��stack���� path������·��
/////Robot��get_number ,pull_number�ж������޸�

#include <bits/stdc++.h> // �����׼���ͷ�ļ�
#include"a_star.h"
using namespace std; // ʹ�ñ�׼�����ռ�

const int n = 200; // ���峣��nΪ200
const int robot_num = 10; // ���峣��robot_numΪ10
const int berth_num = 10; // ���峣��berth_numΪ10
const int N = 210; // ���峣��NΪ210
int boats_laststate[5] = { 1,1,1,1,1 };//���ڼ�¼�������ϸ�ѭ����״̬���ڴ����жϲ������˺�Ż���£�����¼��ǰѭ����״̬

int Money, Capacity, FrameId;
//�����ͼ�ṹ��
char Maps[205][205];
//���巽��
enum dir {
	d_right,
	d_left,
	d_up,
	d_down
};

typedef list<int> pp;//����int��˫����������Ϊ pp;


//���������Ѱ·�ṹ��
struct  MAP
{
	pp find_path;
	pp::iterator current_;
	MAP() {
		current_ = find_path.end();
	}
};

// ���崬ֻ�ṹ��
struct Boat
{
	int state; // ��ֻ״̬
	int berthId; // Ŀ�겴λID(ϵͳ����״̬)
	int arrive_id;//��¼������ʱ��֡id
	int ship[2];//ship[0]:0-��ship 1-ִ��ship;ship[1]:Ŀ�겴λid	
	int go_number;//Ϊ1ʱִ��go
	int goods_loaded;//��¼�Ѿ�����ȥ�Ļ�������
	Boat(int state = 1, int berthId = -1, int arrive_id = 1, int ship0 = 0, int ship1 = 0, int go_number = 0, int goods_loaded = 0)
	{
		this->state = state;
		this->berthId = berthId;
		this->arrive_id = arrive_id;
		ship[0] = ship0;
		ship[1] = ship1;
		this->go_number = go_number;
		this->goods_loaded = goods_loaded;
	}
};

// �������ṹ��
struct Good
{
	int x, y, val; // ��������ͼ�ֵ
	int lossFrameid; // ��ʧʱ�̵�֡��
	int state; // ״̬��������Ϊ0������Ϊ1
	int disToBerth = 0;
	Good() { // Ĭ�Ϲ��캯��
		lossFrameid = FrameId + 1000; // Ĭ������Ϊ1000
		state = 1; // Ĭ��״̬Ϊ1
	}
};


// ���岴λ�ṹ��
struct Berth
{
	int x; // ��λx����
	int y; // ��λy����
	int transport_time; // ����ʱ��
	int loading_speed; // װ���ٶ�
	unordered_set<int> dir;//���ķ���
	int count = 0;
	bool bind = 0;//�ж��Ƿ�󶨻�����
	int ship_exist = 0;//1-�Ѿ��д�Ҫȥ���Ѿ����ڴ���	0-û�д�Ҫȥ��û�д�ͣ��	���뿪��λʱ�ᱻ��0���д���ʱ�ᱻ��1
	vector<Good> goods;
	Berth() {} // Ĭ�Ϲ��캯��
	/*Berth(int x=0, int y=0, int transport_time=0, int loading_speed=0,	int count=0, int ship_exist=0)
	{ // �������Ĺ��캯��
		this->x = x;
		this->y = y;
		this->transport_time = transport_time;
		this->loading_speed = loading_speed;
		this->count = count;
		this->ship_exist = ship_exist;
	}*/
};

// ��������˽ṹ��
struct Robot
{
	/*�ɿ��������������˵���Ϣ*/
	int x, y, goods; // �����������Я���������
	int status; // ״̬
	int mbx, mby; // Ŀ������
	MAP path;//��ͼ 
	int berthId = -1;

	/*������֮����������������Ϣ*/
	int task;	//������ʱ task=1,ִ��pull������
	Robot(int startX = 0, int startY = 0, int _goods = 0, int _status = 0, int _mbx = 0, int _mby = 0, int _task = 0) { // �������Ĺ��캯��
		x = startX;
		y = startY;
		status = _status;
		mbx = _mbx;
		mby = _mby;
		task = _task;

	}
	//����getָ��-0����ִ�У�1����ִ��
	bool getnumber()
	{//��� ������û�л������� �� ����������Ŀ�������غ�ִ��get||��ͼ����ִ��
		if (((task == 2) && (goods == 0) && (x == mbx) && (y == mby)))//״̬0��2����ʾ�Ҷ����ߵ�·���Ѿ����꣬����ִ��get()
		{

			return true;
		}
		else
		{//����ִ��get
			return false;
		}
	}
	//����pullָ��-0����ִ�У�1����ִ��
	int pullnumber()
	{//��� �л������� �� ����������Ŀ�������غ�ִ��pull
	//״̬1��4����ʾ�Ѿ��ͻ���·��·���Ѿ����꣬����ִ��pull()
		if ((task == 4) && (goods == 1) && (x == mbx) && (y == mby))
		{
			return 1;
		}
		else
		{//����ִ��
			return 0;
		}
	}

	//movenumber:-1-���ƶ�/0-����/1-����/2-����/3-����
	int movenumber()
	{
		if ((x == mbx) && (y == mby))
		{//�����Ŀ��λ���غϣ�����
			return -1;
		}
		else
		{//�����Ŀ��λ�ò�ͬ����A*�㷨�Ľ��(����0/1/2/3)��		���������������ԭ��������ʱҲ�������µ����꣬���Ǻ;ɵ�Ŀ��λ���غϣ����ö���������ʱ�ɵ�Ŀ������ᱻ�µĸ���
			int number;
			if (path.current_ != path.find_path.end()) {
				number = *path.current_;   //�����Ŀ��λ�ò�ͬ����A*�㷨�Ľ��(����0/1/2/3)��		���������������ԭ��������ʱҲ�������µ����꣬���Ǻ;ɵ�Ŀ��λ���غϣ����ö���������ʱ�ɵ�Ŀ������ᱻ�µĸ���
				path.current_++;
			}//�����ջ     //return A_path();
			else number = -1;        //û��·���滮��ԭ�ز���
			return number;
		}
	}
	void A_path() {
		//������������
		a_star a({ x,y }, { mbx,mby });  //���ر�����ÿ��������������������
		path.find_path = a.a_star_search(Maps);//���ݵ�ǰ��ͼѰ·�����ػ�����Ѱ·���ƶ������
		path.current_ = path.find_path.begin();
	}
};
//����涨�������Լ۱ȵķ�ʽ��������������Ѱ���Լ۱ȸߵĻ��
bool cmp_dis_val(Good good1, Good good2) {
	float xingjiabi1 = good1.val / (float)(good1.disToBerth * 2);
	float xingjiabi2 = good2.val / (float)(good2.disToBerth * 2);
	return xingjiabi1 > xingjiabi2;
}
// ��������
struct Controller
{
	int frameId, money, capcity; // ֡ID����Ǯ������
	// ��ͼ
	vector<Robot> robots; // ����������
	map<int, Berth> berths; // ��λӳ��
	vector<Boat> boats; // ��ֻ����
	int max_time = 0;
	// ���������캯��
	Controller()
	{
		robots.resize(10); // ����������������СΪ10
		boats.resize(5); // ������ֻ������СΪ5
	}
	/*********************************��ȡ��Ϣ�ຯ��*************************************************************************/

	// ��ȡ��ͼ
	void MapsRead()
	{
		int num = 0;
		for (int i = 0; i < 200; i++) {
			scanf("%s", Maps[i]); // ���ж�ȡ��ͼ��Ϣ
			for (int j = 0; j < 200; j++) {
				if (Maps[i][j] == robot) {

					robots[num].x = i;
					robots[num].y = j;
					++num;
				}
			}
		}

	}

	// ��ȡ��λ�ʹ�ֻ������Ϣ
	void berth_boatCapcity_read()
	{
		for (int l = 0; l < 10; l++)
		{ // ��1��9ѭ��
			struct Berth berth; // ������λ����
			int id; // ��λID
			scanf("%d%d%d%d%d", &id, &berth.x, &berth.y, &berth.transport_time, &berth.loading_speed); // ��ȡ��λ��Ϣ
			//�洢���ķ���
			int i = berth.x, j = berth.y;
			vector<pair<int, int>> edge = { {d_up,0},{d_left,0},{d_right,0},{d_down,0} };
			for (int n = 0; n < 4; n++) {
				if (i - 1 > 0 && Maps[i - 1][j + n] == road)
					edge[0].second++;
				if (j - 1 > 0 && Maps[i + n][j - 1] == road)
					edge[1].second++;
				if (Maps[i + n][j + 4] == road)
					edge[2].second++;
				if (Maps[i + 4][j + n] == road)
					edge[3].second++;
			}
			for (int m = 0; m < edge.size(); m++)
			{
				if (edge[m].second > 3)
					berth.dir.emplace(edge[m].first);

			}

			berths[id] = berth; // ����λ��Ϣ����ӳ����

		}
		for (int j = 0; j < 10; j++) {
			int robotid = -1, mindis = 400;
			for (int i = 0; i < 10; i++) {//�������
				if (robots[i].berthId > -1) continue;//���˾��������沽��
				int dis = abs(robots[i].x - berths[j].x) + abs(robots[i].y - berths[j].y);
				for (auto one_dir = berths[j].dir.begin(); one_dir != berths[j].dir.end(); ++one_dir) {
					switch (*one_dir) {//�ҽ��Ĳ�λ�����㲴λ�ķ�λ�Ͱ�
					case d_down:
						if (robots[i].x > berths[j].x + 3 && dis < mindis) {
							mindis = dis;
							robotid = i;
						}
						break;
					case d_up:
						if (robots[i].x < berths[j].x && dis < mindis)
						{
							mindis = dis;
							robotid = i;
						}
						break;
					case d_left:
						if (robots[i].y < berths[j].y && dis < mindis)
						{
							mindis = dis;
							robotid = i;
						}
						break;
					case d_right:
						if (robots[i].y > berths[j].y + 3 && dis < mindis)
						{
							mindis = dis;
							robotid = i;
						};
						break;
					default:
						break;
					}
				}
			}
			if (robotid >= 0) {
				a_star r_b({ robots[robotid].x,robots[robotid].y }, { berths[j].x,berths[j].y });
				r_b.a_star_search(Maps);
				if (r_b.find_path.front() == -1) break;
				robots[robotid].berthId = j;
				berths[j].bind = true;
			}


		}
		for (int j = 0; j < 10; j++) {
			if (!berths[j].bind) {
				for (int i = 0; i < 10; i++) {
					if (robots[i].berthId < 0) {
						a_star r_b({ robots[i].x,robots[i].y }, { berths[j].x,berths[j].y });
						r_b.a_star_search(Maps);
						if (r_b.find_path.front() == -1) break;
						robots[i].berthId = j;
						berths[j].bind = true;
					}
				}
			}
		}
		scanf("%d", &capcity); // ��ȡ��ֻ������Ϣ
		Capacity = this->capcity; // ����ȫ������
		char judge_end_OK[5]; // �����ж��Ƿ��ȡ���
		scanf("%s", judge_end_OK); // ��ȡ�жϱ�־
		printf("OK\n"); // ���OK
		fflush(stdout); // ������������
	}
	// ��ȡ֡ID�ͽ�Ǯ��Ϣ
	void frameId_money_read()
	{
		scanf("%d%d", &frameId, &money); // ��ȡ֡ID�ͽ�Ǯ��Ϣ
		FrameId = this->frameId; // ����ȫ��֡ID
		Money = this->money; // ����ȫ�ֽ�Ǯ
	}

	// ��ȡ������Ϣ
	void goodsRead()
	{
		int num; // ��������
		scanf("%d", &num); // ��ȡ��������

		if (num != 0) {
			for (int i = 0; i < num; i++)
				
			{ // ѭ����ȡ������Ϣ
				//��ȡ����֮ǰ��������ʧ�Ļ���

				struct Good good; // �����������
				scanf("%d%d%d", &good.x, &good.y, &good.val); // ��ȡ��������ͼ�ֵ
				if(good.y>=197) break;
				//���жϻ����Ƿ�ɷ���ò�λ�����ж�����Ĳ�λ
				int id = -1, mindis = 400;
				for (int i = 0; i < 10; i++) {
					if (berths[i].goods.size() != 0) {
						//�������ӵ�ǰ֡��С����ʧ֡���Ļ�������˻�����ȥѰ�һ����˷�ʱ�䣬
						//��Ȼ�������覴ã��������жϰ�
						auto it = berths[i].goods.begin();
						while (it != berths[i].goods.end()) {
							if (it->lossFrameid + it->disToBerth <= FrameId) it = berths[i].goods.erase(it);
							else it++;
						}
					}
					int dis = abs(good.x - berths[i].x) + abs(good.y - berths[i].y);
					for (auto one_dir = berths[i].dir.begin(); one_dir != berths[i].dir.end(); ++one_dir) {
						switch (*one_dir) {
						case d_down:
							if (good.x > berths[i].x + 3 && dis < mindis) {
								id = i;
								mindis = dis;
							}
							break;
						case d_up:
							if (good.x < berths[i].x && dis < mindis)
							{
								id = i;
								mindis = dis;
							}
							break;
						case d_left:
							if (good.y < berths[i].y && dis < mindis)
							{
								id = i;
								mindis = dis;
							}
							break;
						case d_right:
							if (good.y > berths[i].y + 3 && dis < mindis)
							{
								id = i;
								mindis = dis;
							}
							break;
						default:
							break;
						}
					}

				}
				//�벴λ
				if (id >= 0) {
					good.disToBerth = mindis;
					berths[id].goods.push_back(good);
				}
				sort(berths[id].goods.begin(), berths[id].goods.end(), cmp_dis_val);//����������
			}
		}
	}

	// ��ȡ��������Ϣ
	void robotRead()
	{
		for (int i = 0; i < 10; i++)
		{ // ѭ����ȡ��������Ϣ
			int x, y;
			scanf("%d%d%d%d", &robots[i].goods, &x, &y, &robots[i].status); // ��ȡ��������Ϣ
			Maps[x][y] = robot; //���µ�ͼ�ϻ����˱�ʾ
			robots[i].x = x;
			robots[i].y = y;

		}
	}

	// ��ȡ��ֻ��Ϣ
	void boatRead()
	{
		for (int i = 0; i < 5; i++)
		{ // ѭ����ȡ��ֻ��Ϣ
			scanf("%d%d", &boats[i].state, &boats[i].berthId); // ��ȡ��ֻ��Ϣ
		}
		char okk[100];
		scanf("%s", okk); // ��ȡ��ȷ���������
	}
	//max_time��ʼ��
	void maxtime_init()
	{
		for (int i = 0; i < 10; i++)
		{//����ÿ����λ,�ҳ��������ʱ��
			if (berths[i].transport_time > max_time)
			{
				max_time = berths[i].transport_time;
			}
		}
	}
	/*********************************�����ຯ��*************************************************************************/
	//���ݻ�����״̬�������˷���Ŀ�����꣬�����ǻ���Ҳ�����ǲ�λ
   //������ʵ�����µĺ������ɺ���ɾ��

	loca goods_assign(int i) {
		if (berths[robots[i].berthId].goods.size() != 0) {
			struct Good good = berths[robots[i].berthId].goods[0];//���׵Ļ�����г�Ϊ������Ŀ�ꡣ
			loca l = { good.x,good.y };
			auto it = berths[robots[i].berthId].goods.begin();
			it = berths[robots[i].berthId].goods.erase(it);
			return l;
		}
		else return { -1,0 };
	}
	//************************************************************************************************************
	//************************************************************************************************************
	//ʵ����
	void _robot_assign()
	{
		for (int i = 0; i < 10; i++)
		{ // ѭ����ȡ��������Ϣ
			if (robots[i].status == 1) {   //�����������
				if (robots[i].goods == 0 && robots[i].task == 0)
				{	//״̬0��0������û����
					//û������ʱ�����Ӧ����Ļ������꣬����task��1
					auto L = goods_assign(i); //�ӻ���������ȡ����������
					if (L.first != -1) {     //�����������л���
						robots[i].mbx = L.first;
						robots[i].mby = L.second;  //�������긳��������Ŀ������
						robots[i].task = 1;
						robots[i].path.find_path.clear(); //���·��
						robots[i].path.current_ = robots[i].path.find_path.end();
						robots[i].A_path();   //��ȡ��Ŀ��·��
					}
					else robots[i].task = 0;  //����������û���������0��0̬���ȷ������
				}
				if (robots[i].goods == 0 && robots[i].task == 2) {
					//Ŀǰ��״̬Ϊ0��2 ��ζ���ߴ�����ߵ�û��get��
					robots[i].task = 0;    //�ص�0��0�ȴ����·���Ŀ��

				}
				if (robots[i].goods == 1 && robots[i].task == 2) {
					// Ŀǰ״̬Ϊ1��2����ζ���ߵ��˲���get���˻�����䲴λ���ꣻ
					//�����ͻ�״̬ 1��3
					robots[i].mbx = berths[robots[i].berthId].x;
					robots[i].mby = berths[robots[i].berthId].y;
					robots[i].task = 3;
					robots[i].path.find_path.clear(); //���·��
					robots[i].path.current_ = robots[i].path.find_path.end();
					robots[i].A_path();

				}
				//Ŀǰ״̬1��4 ��ζ���������Ҳ�λ������û���»������·��䲴λ
				if (robots[i].goods == 1 && robots[i].task == 4) {
					robots[i].task = 2;   //�ص�״̬1��2�����»�ȡ��λ


				}
				//Ŀǰ״̬0��4 ��ζ���ҵ���λ��������
				if (robots[i].goods == 0 && robots[i].task == 4) {
					robots[i].path.find_path.clear(); //���·��
					robots[i].path.current_ = robots[i].path.find_path.end();

					robots[i].task = 0; //�ص�״̬0��0�ȴ����·������

				}


			}
			///////////////////////////
			else { //��ʾ������ײ

				robots[i].path.find_path.clear(); //���·��
				robots[i].path.current_ = robots[i].path.find_path.end();
				if (robots[i].goods == 1) {
					robots[i].task = 2;  //������˻�״̬��ص�1��2���¹滮��λ·��
				}
				else robots[i].task = 0;//�������¹滮ȡ��·��
			}
			///////////////////////////////��ײ���Ʋ������ƣ���Ҫ�ھ��忼���Ż�
		}
	}

	void _output_get()
	{
		for (int i = 0; i < 10; i++)
		{//�������л�����
			if (robots[i].getnumber())
			{//�����������һ��������get�ź���1����ִ��getָ��
				printf("%s %d \n", "get", i);
			}

		}
	}
	//���pullָ��
	void _output_pull()
	{
		for (int i = 0; i < 10; i++)
		{//�������л�����
			if (robots[i].pullnumber() == 1)
			{//�����������һ����pull�ź���1����ִ��pullָ��
				printf("%s %d \n", "pull", i);
				berths[robots[i].berthId].count++;
			}
		}
	}
	void _output_move()
	{
		for (int i = 0; i < 10; i++)

		{//�������л�����
			int n = robots[i].movenumber();
			if (n != -1)
			{//�����������һ����move����-1��������ԭ�ز�������ִ��moveָ��
				printf("move %d %d\n", i, n);
				Maps[robots[i].x][robots[i].y] = road;
			}

			else {  //��-1�������Ѿ�������·�̻���û������鵱ǰ״̬
				if (robots[i].goods == 0 && robots[i].task == 1) {
					robots[i].task = 2;                 //��0��1״̬�������һ���·���Ѿ����꣬ת��1��2ִ̬��get
				}
				if (robots[i].goods == 1 && robots[i].task == 3) {
					robots[i].task = 4;                 //��0��1״̬�������ͻ�����λ·���Ѿ����꣬ת��1��4ִ̬��pull
				}
			};
		}
	}
	//���ȴ����������״̬�Ĵ����䲴λĿ��
	void ship_number()//Ϊ1ʱִ��ship
	{//����ÿ����
		for (int i = 0; i < 5; i++)
		{
			if ((boats[i].state == 2) || (boats[i].state == 1 && (boats[i].berthId == -1)))//�����ʱ���������״̬��ͣ�ڲ�λ�����ж�Ҫִ��ship
			{
				int max_goodsnumber = 0;//��λ������
				int berth_id = -1;
				for (int j = 0; j < 10; j++)
				{//����ÿ����λ
					if (berths[j].ship_exist == 0 && berths[j].count >= max_goodsnumber)//�����ò�λ�� "ѡ�л�����/��λtime"���� ��ΪĿ��
					{
						berth_id = j;
						max_goodsnumber = berths[j].count;
					}
				}
				if (berth_id != -1)//����з������������ò�λ,��ship�źź�Ŀ�겴λid���͸���,����Ŀ�겴λ��ship_exist��1
				{
					boats[i].ship[0] = 1;
					boats[i].ship[1] = berth_id;
					berths[berth_id].ship_exist = 1;
				}
				else
				{//�����ʱû�����ò�λ��ship�ź���0�����ô�����
					boats[i].ship[0] = 0;
				}

			}
			else
			{//�����ǿ���ʱ��ִ��ship
				boats[i].ship[0] = 0;
			}
		}
	}

	void go_number()
	{
		for (int i = 0; i < 5; i++)
		{
			// ��ǰ�᣺�����ʱstate==1&&boats[i].berthId != -1(�ڲ�λ)
			if (boats[i].state == 1 && boats[i].berthId != -1)
			{
				//�ж��ϸ�ѭ����״̬�ǲ���1������ϸ�ѭ����״̬��1��˵����ʱ��װ��;��
				if (boats_laststate[i] == 1)
				{//�жϴӴ��ĵ���֡����ǰ֡�����װ���ٻ�(��λ��һ������ô��)
					int goods_count = (FrameId - boats[i].arrive_id) * berths[boats[i].berthId].loading_speed;
					if (berths[boats[i].berthId].count >= goods_count)
					{//�����λ�ϵ�ǰ�����������ڵ���goods_count��������ൽû���ü�����
						if (goods_count > (Capacity - boats[i].goods_loaded))
						{//���goods_count>����ʣ��������˵���Ӵ��ĵ���֡����ǰ�Ѿ�װ����
							boats[i].go_number = 1;//���װ�Ļ��������������ˣ���ô���Գ���
							berths[boats[i].berthId].count = berths[boats[i].berthId].count - (Capacity - boats[i].goods_loaded);//���²�λ����
							berths[boats[i].berthId].ship_exist = 0;//��λ��Ϊ����
							boats[i].goods_loaded = 0;//�Ѿ���������go�ź��ˣ������Ѿ��ػ���Ϊ0
						}
						else
						{//���û��ʣ�������ͼ����ȣ�������
							boats[i].go_number = 0;
						}
					}
					else
					{//��λ�ϵ�ǰ��������С��goods_count,��װ���پ��Ѿ�װ�˶���
						if (berths[boats[i].berthId].count < (Capacity - boats[i].goods_loaded))
						{//�����λ������С�ڴ���ʣ��������˵����Щȫ��װ���˵�Ҳûװ��
							if ((boats[i].goods_loaded + berths[boats[i].berthId].count) < (Capacity / 2))
							{//�����װ�˲���һ��Ļ�����ʱ������ȥ����㣬���Ǵ������еĻ���
								boats[i].go_number = 0;
								boats[i].goods_loaded = boats[i].goods_loaded + berths[boats[i].berthId].count;//����Я���Ļ�������
								berths[boats[i].berthId].count = 0;//���µ�ǰ��λ�ϵĻ�������Ϊ��
								//��Ѱ���µĲ�λ
								int max_goodscount = 0;//��λ������
								int berth_id = -1;
								for (int j = 0; j < 10; j++)
								{//����ÿ����λ
									if (berths[j].ship_exist == 0 && berths[j].count >= max_goodscount)//�����ò�λ�� ѡ�������Ĳ�λ ��ΪĿ��
									{
										berth_id = j;
										max_goodscount = berths[j].count;
									}
								}
								berths[boats[i].berthId].ship_exist = 0;//��ֹ�ѵ�ǰ���ڲ�λ�ֵ�����һ��Ŀ�꣬���Դ�ʱ�ŰѸò�λ��Ϊ����
								if (berth_id != -1)//����з������������ò�λ,��ship�źź�Ŀ�겴λid���͸���,���´��� ���ػ���Ϊ���ϵ�ǰ��λ�ϵ�����������Ŀ�겴λ��ship_exist��1
								{
									boats[i].ship[0] = 1;
									boats[i].ship[1] = berth_id;
									berths[berth_id].ship_exist = 1;
								}
								else
								{//�����ʱû�����ò�λ��ship�ź���0�����ô�����
									boats[i].ship[0] = 0;
								}
							}
							else
							{//������������λ(��֮ǰ�����������Ĳ�λ)�Ļ�����������װ��һ�������ˣ��Ǿ�ȥ�����
								boats[i].go_number = 1;//����
								berths[boats[i].berthId].count = 0;//���²�λ����
								berths[boats[i].berthId].ship_exist = 0;//��λ��Ϊ����
								boats[i].goods_loaded = 0;//�Ѿ���������go�ź��ˣ������Ѿ��ػ���Ϊ0
							}
						}
						else
						{//��λ���������ڵ��ڴ���ʣ��������˵��װ���ˣ�����go
							boats[i].go_number = 1;
							berths[boats[i].berthId].count = berths[boats[i].berthId].count - (Capacity - boats[i].goods_loaded);//���²�λ����
							boats[i].goods_loaded = 0;//����Ҫgo�ˣ���Я������������
							berths[boats[i].berthId].ship_exist = 0;//��λ��Ϊ����
						}
					}
				}
				else
				{//����ϸ�״̬��0��2����ʱ״̬��1��˵����ʱ�ϸ�ѭ��-���ѭ���Ĺ����У���ǡ�������ѭ�������ˣ�
				//��ʱ�����ǵ�ǰѭ��ǡ�õ�����¼��ǰ֡id��������������ʱ��֡id
					boats[i].arrive_id = FrameId;
					boats[i].go_number = 0;
				}
			}

			else
			{//û�ڲ�λ
				boats[i].go_number = 0;
			}
		}
	}

	void laststate_record()
	{
		for (int i = 0; i < 5; i++)
		{
			boats_laststate[i] = boats[i].state;
		}
	}
	void output_ship()
	{
		for (int i = 0; i < 5; i++)
		{
			if (boats[i].ship[0] == 1)
			{
				printf("%s %d %d \n", "ship", i, boats[i].ship[1]);
			}
		}
	}
	void output_go()
	{
		for (int i = 0; i < 5; i++)
		{
			if (boats[i].go_number == 1)
			{
				printf("%s %d\n", "go", i);
			}
		}
	}

	void retreat()
	{
		if (FrameId >= (15000 - max_time))
		{
			for (int i = 0; i < 5; i++)
			{//����ÿ�Ҵ�
				if (FrameId>=(15000 - berths[boats[i].berthId].transport_time - 10))
				{//�ٲ�run����������
					boats[i].ship[0] = 0;
					boats[i].go_number = 1;
				}
			}
		}
	}
	//************************************************************************************************************

};

int main() {
	Controller controller; // ��������������
	controller.MapsRead(); // ��ȡ��ͼ��Ϣ
	controller.berth_boatCapcity_read(); // ��ȡ��λ�ʹ�ֻ������Ϣ
	controller.maxtime_init();//��ʼ��maxtime
	for (int zhen = 1; zhen <= 15000; zhen++) // ѭ�����п���
	{/*��ȡ���洢����*/
		controller.frameId_money_read(); // ��ȡ֡ID�ͽ�Ǯ��Ϣ
		controller.goodsRead(); // ��ȡ������Ϣ
		controller.robotRead(); // ��ȡ��������Ϣ
		controller.boatRead(); // ��ȡ��ֻ��Ϣ
		/*�м����*/
		controller._robot_assign();//�������˷���Ŀ��λ��
		controller.ship_number();//�����õĴ�����Ŀ�겴λ
		controller.go_number();//��������go�ź�
		controller.retreat();//�ж�Ҫ��Ҫȫ������
		controller.laststate_record();//��¼���ĵ�ǰѭ��״̬��Ϊ�¸�ѭ����laststate
		//��������Ŀ�겴λ
/*���*/

	//������ָ��
		controller._output_move();
		controller._output_get();
		controller._output_pull();   //�º���ָ��
		//��ָ��
		controller.output_ship();
		controller.output_go();

		puts("OK"); // ���OK
		fflush(stdout); // ������������
	}
	return 0; // ����0����ʾ������������
}
