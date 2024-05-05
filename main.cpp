#define _CRT_SECURE_NO_WARNINGS
/*
* Design by Wang.jh.
* Based on "demo.cpp" by Huang.wj
*
* 新增功能描述：
*
* 1.控制器结构体新增：
*	控制类成员函数”：
*	  robot_assign()-根据机器人状态 有无任务/是否携带货物，根据不同的状态为所有机器人发送货物或泊位的目标位置
*	  output_move()-用output_move输出所有机器人的move指令
*     output_get()-输出所有机器人的get指令，机器人的判断如果是不get，则不输出该机器人的get指令
*	  output_pull()-同上

* 2.机器人结构体新增：
*	反馈给控制器的成员变量：
*	  task：0-无任务 1-有任务（寻找货物/寻找泊位）	执行pull以后会将task清零,控制器发现task=0的机器人时会在分配目标的同时将task置1
*   成员函数：
*	  int movenumber():机器人坐标与目标坐标相同时，无论有无任务都不能动；与目标位置不同时无论task都根据A*朝目标前进
*	  int getnumber():有任务情况下，没有携带货物 且 自身坐标与目标坐标重合执行get
*	  int pullnumber():携带货物，且与目标重合时执行pull
*
* 3.补充说明：
* 分配特定区域物品坐标的算法未完成
* 寻址算法未完成
* 机器人的逻辑只能是一帧完成一个动作，优点在于 稳定，缺点时相比于在一帧内同时执行move和get的过程会慢一帧
* 修改了Robot的构造函数，改为缺省型便于初始化新增成员变量
* bug：如果赶到目标位置前货物消失了，则会陷入一直get的循环
*
*
* 备选方案：读取存储型路径(map)进行移动方案下的机器人行为逻辑：
* 4.机器人读取存储型路径(map)进行移动的思路：这里需要一个变量i记录map上的路径已经走了几步，接下来执行第i+1步
*							可以考虑定义结构体map，成员变量包含目标位置，规划的路径，记录走到第几步的成员变量i，
*							以及记录是否已经执行get指令的成员变量get_already
* get和pull的判断加一条：“或者map路径走完”。走完了map那么move是不动的，如果路线规划错/货物消失的情况下无论get与否，
*							该机器人不会进入到携带货物的状态，控制器是不会下发到泊位的map()，机器人永远卡死不动。
*							为解决该问题，可以让机器人走完map后get一次，并将get_already=1。
*							控制器会为get_already==1&&goods==0的机器人下发一份新的地图，寻找新的目标，pull类似
* move：如果和目标坐标不同:1.如果判断是 还没走完map的情况则按map走()，
*
*							。如果走完map上的路径，理想情况下机器人坐标会
*							与目标坐标重合，衔接上get与pull的判断；
*
*						   2.如果已经走完地图上的路径，说明规划路线有误，先不动，等待控制器接收到路线错误的指令(由get函数判断发出)后
*								发出新的map
*		如果和目标坐标相同：不动
*
* 补充：建议规划好一个路线后执行到底，下一个目标优先选金额大的
*/

//    3月17号
/////修改函数全在实验区，将Robot中的成员变量MAP结构体删除，只使用stack类型 path来保存路径
/////Robot中get_number ,pull_number判断条件修改

#include <bits/stdc++.h> // 引入标准库的头文件
#include"a_star.h"
using namespace std; // 使用标准命名空间

const int n = 200; // 定义常量n为200
const int robot_num = 10; // 定义常量robot_num为10
const int berth_num = 10; // 定义常量berth_num为10
const int N = 210; // 定义常量N为210
int boats_laststate[5] = { 1,1,1,1,1 };//用于记录船的在上个循环的状态，在船的判断操作完了后才会更新，即记录当前循环的状态

int Money, Capacity, FrameId;
//定义地图结构体
char Maps[205][205];
//定义方向
enum dir {
	d_right,
	d_left,
	d_up,
	d_down
};

typedef list<int> pp;//定义int型双向链表类型为 pp;


//定义机器人寻路结构体
struct  MAP
{
	pp find_path;
	pp::iterator current_;
	MAP() {
		current_ = find_path.end();
	}
};

// 定义船只结构体
struct Boat
{
	int state; // 船只状态
	int berthId; // 目标泊位ID(系统给的状态)
	int arrive_id;//记录船到达时的帧id
	int ship[2];//ship[0]:0-不ship 1-执行ship;ship[1]:目标泊位id	
	int go_number;//为1时执行go
	int goods_loaded;//记录已经搬上去的货物数量
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

// 定义货物结构体
struct Good
{
	int x, y, val; // 货物坐标和价值
	int lossFrameid; // 消失时刻的帧数
	int state; // 状态，被拿了为0，否则为1
	int disToBerth = 0;
	Good() { // 默认构造函数
		lossFrameid = FrameId + 1000; // 默认寿命为1000
		state = 1; // 默认状态为1
	}
};


// 定义泊位结构体
struct Berth
{
	int x; // 泊位x坐标
	int y; // 泊位y坐标
	int transport_time; // 运输时间
	int loading_speed; // 装载速度
	unordered_set<int> dir;//海的方向
	int count = 0;
	bool bind = 0;//判断是否绑定机器人
	int ship_exist = 0;//1-已经有船要去或已经存在船了	0-没有船要去且没有船停靠	船离开泊位时会被置0，有船来时会被置1
	vector<Good> goods;
	Berth() {} // 默认构造函数
	/*Berth(int x=0, int y=0, int transport_time=0, int loading_speed=0,	int count=0, int ship_exist=0)
	{ // 带参数的构造函数
		this->x = x;
		this->y = y;
		this->transport_time = transport_time;
		this->loading_speed = loading_speed;
		this->count = count;
		this->ship_exist = ship_exist;
	}*/
};

// 定义机器人结构体
struct Robot
{
	/*由控制器发给机器人的信息*/
	int x, y, goods; // 机器人坐标和携带货物与否
	int status; // 状态
	int mbx, mby; // 目标坐标
	MAP path;//地图 
	int berthId = -1;

	/*机器人之后反馈给控制器的信息*/
	int task;	//有任务时 task=1,执行pull后清零
	Robot(int startX = 0, int startY = 0, int _goods = 0, int _status = 0, int _mbx = 0, int _mby = 0, int _task = 0) { // 带参数的构造函数
		x = startX;
		y = startY;
		status = _status;
		mbx = _mbx;
		mby = _mby;
		task = _task;

	}
	//返回get指令-0代表不执行，1代表执行
	bool getnumber()
	{//如果 有任务，没有货物在身 且 自身坐标与目标坐标重合执行get||地图走完执行
		if (((task == 2) && (goods == 0) && (x == mbx) && (y == mby)))//状态0，2，表示找东西走的路程已经走完，可以执行get()
		{

			return true;
		}
		else
		{//否则不执行get
			return false;
		}
	}
	//返回pull指令-0代表不执行，1代表执行
	int pullnumber()
	{//如果 有货物在身 且 自身坐标与目标坐标重合执行pull
	//状态1，4，表示已经送货的路策路程已经走完，可以执行pull()
		if ((task == 4) && (goods == 1) && (x == mbx) && (y == mby))
		{
			return 1;
		}
		else
		{//否则不执行
			return 0;
		}
	}

	//movenumber:-1-不移动/0-右移/1-左移/2-上移/3-下移
	int movenumber()
	{
		if ((x == mbx) && (y == mby))
		{//如果和目标位置重合，不动
			return -1;
		}
		else
		{//如果和目标位置不同，按A*算法的结果(返回0/1/2/3)走		不考虑有无任务的原因：无任务时也不会有新的坐标，还是和旧的目标位置重合，不用动；有任务时旧的目标坐标会被新的覆盖
			int number;
			if (path.current_ != path.find_path.end()) {
				number = *path.current_;   //如果和目标位置不同，按A*算法的结果(返回0/1/2/3)走		不考虑有无任务的原因：无任务时也不会有新的坐标，还是和旧的目标位置重合，不用动；有任务时旧的目标坐标会被新的覆盖
				path.current_++;
			}//命令出栈     //return A_path();
			else number = -1;        //没有路径规划在原地不动
			return number;
		}
	}
	void A_path() {
		//将机器人坐标
		a_star a({ x,y }, { mbx,mby });  //本地变量，每次搜索后会清除所有数据
		path.find_path = a.a_star_search(Maps);//根据当前地图寻路并返回机器人寻路的移动命令号
		path.current_ = path.find_path.begin();
	}
};
//排序规定，按照性价比的方式给货物排序，优先寻找性价比高的货物。
bool cmp_dis_val(Good good1, Good good2) {
	float xingjiabi1 = good1.val / (float)(good1.disToBerth * 2);
	float xingjiabi2 = good2.val / (float)(good2.disToBerth * 2);
	return xingjiabi1 > xingjiabi2;
}
// 控制器类
struct Controller
{
	int frameId, money, capcity; // 帧ID、金钱、容量
	// 地图
	vector<Robot> robots; // 机器人向量
	map<int, Berth> berths; // 泊位映射
	vector<Boat> boats; // 船只向量
	int max_time = 0;
	// 控制器构造函数
	Controller()
	{
		robots.resize(10); // 调整机器人向量大小为10
		boats.resize(5); // 调整船只向量大小为5
	}
	/*********************************读取信息类函数*************************************************************************/

	// 读取地图
	void MapsRead()
	{
		int num = 0;
		for (int i = 0; i < 200; i++) {
			scanf("%s", Maps[i]); // 逐行读取地图信息
			for (int j = 0; j < 200; j++) {
				if (Maps[i][j] == robot) {

					robots[num].x = i;
					robots[num].y = j;
					++num;
				}
			}
		}

	}

	// 读取泊位和船只容量信息
	void berth_boatCapcity_read()
	{
		for (int l = 0; l < 10; l++)
		{ // 从1到9循环
			struct Berth berth; // 创建泊位对象
			int id; // 泊位ID
			scanf("%d%d%d%d%d", &id, &berth.x, &berth.y, &berth.transport_time, &berth.loading_speed); // 读取泊位信息
			//存储海的方向
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

			berths[id] = berth; // 将泊位信息存入映射中

		}
		for (int j = 0; j < 10; j++) {
			int robotid = -1, mindis = 400;
			for (int i = 0; i < 10; i++) {//遍机器人
				if (robots[i].berthId > -1) continue;//绑定了就跳过下面步骤
				int dis = abs(robots[i].x - berths[j].x) + abs(robots[i].y - berths[j].y);
				for (auto one_dir = berths[j].dir.begin(); one_dir != berths[j].dir.end(); ++one_dir) {
					switch (*one_dir) {//找近的泊位，满足泊位的方位就绑定
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
		scanf("%d", &capcity); // 读取船只容量信息
		Capacity = this->capcity; // 更新全局容量
		char judge_end_OK[5]; // 用于判断是否读取完成
		scanf("%s", judge_end_OK); // 读取判断标志
		printf("OK\n"); // 输出OK
		fflush(stdout); // 清空输出缓冲区
	}
	// 读取帧ID和金钱信息
	void frameId_money_read()
	{
		scanf("%d%d", &frameId, &money); // 读取帧ID和金钱信息
		FrameId = this->frameId; // 更新全局帧ID
		Money = this->money; // 更新全局金钱
	}

	// 读取货物信息
	void goodsRead()
	{
		int num; // 货物数量
		scanf("%d", &num); // 读取货物数量

		if (num != 0) {
			for (int i = 0; i < num; i++)
				
			{ // 循环读取货物信息
				//读取货物之前先清理消失的货物

				struct Good good; // 创建货物对象
				scanf("%d%d%d", &good.x, &good.y, &good.val); // 读取货物坐标和价值
				if(good.y>=197) break;
				//先判断货物是否可放入该泊位，在判断最近的泊位
				int id = -1, mindis = 400;
				for (int i = 0; i < 10; i++) {
					if (berths[i].goods.size() != 0) {
						//清楚距离加当前帧数小于消失帧数的货物，避免了机器人去寻找货物浪费时间，
						//当然还会存在瑕疵，先这样判断吧
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
				//入泊位
				if (id >= 0) {
					good.disToBerth = mindis;
					berths[id].goods.push_back(good);
				}
				sort(berths[id].goods.begin(), berths[id].goods.end(), cmp_dis_val);//给货物排序
			}
		}
	}

	// 读取机器人信息
	void robotRead()
	{
		for (int i = 0; i < 10; i++)
		{ // 循环读取机器人信息
			int x, y;
			scanf("%d%d%d%d", &robots[i].goods, &x, &y, &robots[i].status); // 读取机器人信息
			Maps[x][y] = robot; //更新地图上机器人表示
			robots[i].x = x;
			robots[i].y = y;

		}
	}

	// 读取船只信息
	void boatRead()
	{
		for (int i = 0; i < 5; i++)
		{ // 循环读取船只信息
			scanf("%d%d", &boats[i].state, &boats[i].berthId); // 读取船只信息
		}
		char okk[100];
		scanf("%s", okk); // 读取并确认输入完成
	}
	//max_time初始化
	void maxtime_init()
	{
		for (int i = 0; i < 10; i++)
		{//遍历每个泊位,找出最长的运输时间
			if (berths[i].transport_time > max_time)
			{
				max_time = berths[i].transport_time;
			}
		}
	}
	/*********************************操作类函数*************************************************************************/
	//根据机器人状态给机器人分配目标坐标，可能是货物也可能是泊位
   //已启用实验区新的函数，旧函数删除

	loca goods_assign(int i) {
		if (berths[robots[i].berthId].goods.size() != 0) {
			struct Good good = berths[robots[i].berthId].goods[0];//队首的货物出列成为机器人目标。
			loca l = { good.x,good.y };
			auto it = berths[robots[i].berthId].goods.begin();
			it = berths[robots[i].berthId].goods.erase(it);
			return l;
		}
		else return { -1,0 };
	}
	//************************************************************************************************************
	//************************************************************************************************************
	//实验区
	void _robot_assign()
	{
		for (int i = 0; i < 10; i++)
		{ // 循环读取机器人信息
			if (robots[i].status == 1) {   //如果正常运行
				if (robots[i].goods == 0 && robots[i].task == 0)
				{	//状态0，0，空闲没任务
					//没有任务时分配对应区域的货物坐标，并将task置1
					auto L = goods_assign(i); //从货物数组中取出货物坐标
					if (L.first != -1) {     //货物数组中有货物
						robots[i].mbx = L.first;
						robots[i].mby = L.second;  //货物坐标赋给机器人目的坐标
						robots[i].task = 1;
						robots[i].path.find_path.clear(); //清空路径
						robots[i].path.current_ = robots[i].path.find_path.end();
						robots[i].A_path();   //获取到目标路径
					}
					else robots[i].task = 0;  //货物数组中没货物，保持在0，0态，等分配货物
				}
				if (robots[i].goods == 0 && robots[i].task == 2) {
					//目前的状态为0，2 意味着走错或者走到没有get到
					robots[i].task = 0;    //回到0，0等待重新分配目标

				}
				if (robots[i].goods == 1 && robots[i].task == 2) {
					// 目前状态为1，2，意味着走到了并且get到了货物，分配泊位坐标；
					//进入送货状态 1，3
					robots[i].mbx = berths[robots[i].berthId].x;
					robots[i].mby = berths[robots[i].berthId].y;
					robots[i].task = 3;
					robots[i].path.find_path.clear(); //清空路径
					robots[i].path.current_ = robots[i].path.find_path.end();
					robots[i].A_path();

				}
				//目前状态1，4 意味着走完了找泊位，但是没放下货，重新分配泊位
				if (robots[i].goods == 1 && robots[i].task == 4) {
					robots[i].task = 2;   //回到状态1，2，重新获取泊位


				}
				//目前状态0，4 意味着找到泊位并放下了
				if (robots[i].goods == 0 && robots[i].task == 4) {
					robots[i].path.find_path.clear(); //清空路径
					robots[i].path.current_ = robots[i].path.find_path.end();

					robots[i].task = 0; //回到状态0，0等待重新分配货物

				}


			}
			///////////////////////////
			else { //表示机器人撞

				robots[i].path.find_path.clear(); //清空路径
				robots[i].path.current_ = robots[i].path.find_path.end();
				if (robots[i].goods == 1) {
					robots[i].task = 2;  //如果是运货状态则回到1，2重新规划泊位路线
				}
				else robots[i].task = 0;//否则重新规划取货路线
			}
			///////////////////////////////碰撞机制不够完善，需要在具体考虑优化
		}
	}

	void _output_get()
	{
		for (int i = 0; i < 10; i++)
		{//遍历所有机器人
			if (robots[i].getnumber())
			{//如果机器人这一步发出的get信号是1，就执行get指令
				printf("%s %d \n", "get", i);
			}

		}
	}
	//输出pull指令
	void _output_pull()
	{
		for (int i = 0; i < 10; i++)
		{//遍历所有机器人
			if (robots[i].pullnumber() == 1)
			{//如果机器人这一步的pull信号是1，就执行pull指令
				printf("%s %d \n", "pull", i);
				berths[robots[i].berthId].count++;
			}
		}
	}
	void _output_move()
	{
		for (int i = 0; i < 10; i++)

		{//遍历所有机器人
			int n = robots[i].movenumber();
			if (n != -1)
			{//如果机器人这一步的move不是-1，即不是原地不动，就执行move指令
				printf("move %d %d\n", i, n);
				Maps[robots[i].x][robots[i].y] = road;
			}

			else {  //是-1代表了已经走完了路程或者没动，检查当前状态
				if (robots[i].goods == 0 && robots[i].task == 1) {
					robots[i].task = 2;                 //是0，1状态表明中找货物路程已经走完，转到1，2态执行get
				}
				if (robots[i].goods == 1 && robots[i].task == 3) {
					robots[i].task = 4;                 //是0，1状态表明中送货到泊位路程已经走完，转到1，4态执行pull
				}
			};
		}
	}
	//给等待或运输完成状态的船分配泊位目标
	void ship_number()//为1时执行ship
	{//遍历每个船
		for (int i = 0; i < 5; i++)
		{
			if ((boats[i].state == 2) || (boats[i].state == 1 && (boats[i].berthId == -1)))//如果此时是运输完成状态或停在泊位，则判断要执行ship
			{
				int max_goodsnumber = 0;//泊位上数量
				int berth_id = -1;
				for (int j = 0; j < 10; j++)
				{//遍历每个泊位
					if (berths[j].ship_exist == 0 && berths[j].count >= max_goodsnumber)//从闲置泊位中 "选中货物数/泊位time"最大的 作为目标
					{
						berth_id = j;
						max_goodsnumber = berths[j].count;
					}
				}
				if (berth_id != -1)//如果有符合条件的闲置泊位,则将ship信号和目标泊位id发送给船,并将目标泊位的ship_exist置1
				{
					boats[i].ship[0] = 1;
					boats[i].ship[1] = berth_id;
					berths[berth_id].ship_exist = 1;
				}
				else
				{//如果暂时没有闲置泊位则将ship信号置0，不让船出发
					boats[i].ship[0] = 0;
				}

			}
			else
			{//船不是空闲时不执行ship
				boats[i].ship[0] = 0;
			}
		}
	}

	void go_number()
	{
		for (int i = 0; i < 5; i++)
		{
			// 大前提：如果此时state==1&&boats[i].berthId != -1(在泊位)
			if (boats[i].state == 1 && boats[i].berthId != -1)
			{
				//判断上个循环的状态是不是1，如果上个循环的状态是1，说明此时在装货途中
				if (boats_laststate[i] == 1)
				{//判断从船的到达帧到当前帧最多能装多少货(泊位不一定有这么多)
					int goods_count = (FrameId - boats[i].arrive_id) * berths[boats[i].berthId].loading_speed;
					if (berths[boats[i].berthId].count >= goods_count)
					{//如果泊位上当前货物数量大于等于goods_count，即货物多到没来得及搬完
						if (goods_count > (Capacity - boats[i].goods_loaded))
						{//如果goods_count>船的剩余容量，说明从船的到达帧到当前已经装满了
							boats[i].go_number = 1;//如果装的货超过船的容量了，那么可以出发
							berths[boats[i].berthId].count = berths[boats[i].berthId].count - (Capacity - boats[i].goods_loaded);//更新泊位数量
							berths[boats[i].berthId].ship_exist = 0;//泊位标为闲置
							boats[i].goods_loaded = 0;//已经决定发出go信号了，更新已经载货数为0
						}
						else
						{//如果没到剩余容量就继续等，不出发
							boats[i].go_number = 0;
						}
					}
					else
					{//泊位上当前货物数量小于goods_count,能装多少就已经装了多少
						if (berths[boats[i].berthId].count < (Capacity - boats[i].goods_loaded))
						{//如果泊位上数量小于船的剩余容量，说明这些全都装上了但也没装满
							if ((boats[i].goods_loaded + berths[boats[i].berthId].count) < (Capacity / 2))
							{//如果船装了不到一半的货，此时决定不去虚拟点，而是带上所有的货，
								boats[i].go_number = 0;
								boats[i].goods_loaded = boats[i].goods_loaded + berths[boats[i].berthId].count;//更新携带的货物数量
								berths[boats[i].berthId].count = 0;//更新当前泊位上的货物数量为零
								//并寻找新的泊位
								int max_goodscount = 0;//泊位上数量
								int berth_id = -1;
								for (int j = 0; j < 10; j++)
								{//遍历每个泊位
									if (berths[j].ship_exist == 0 && berths[j].count >= max_goodscount)//从闲置泊位中 选出货最多的泊位 作为目标
									{
										berth_id = j;
										max_goodscount = berths[j].count;
									}
								}
								berths[boats[i].berthId].ship_exist = 0;//防止把当前所在泊位又当作下一个目标，所以此时才把该泊位置为空闲
								if (berth_id != -1)//如果有符合条件的闲置泊位,则将ship信号和目标泊位id发送给船,更新船的 已载货数为加上当前泊位上的数量，并将目标泊位的ship_exist置1
								{
									boats[i].ship[0] = 1;
									boats[i].ship[1] = berth_id;
									berths[berth_id].ship_exist = 1;
								}
								else
								{//如果暂时没有闲置泊位则将ship信号置0，不让船出发
									boats[i].ship[0] = 0;
								}
							}
							else
							{//如果船把这个泊位(来之前货物数量最多的泊位)的货都搬完了且装了一半以上了，那就去虚拟点
								boats[i].go_number = 1;//出发
								berths[boats[i].berthId].count = 0;//更新泊位数量
								berths[boats[i].berthId].ship_exist = 0;//泊位标为闲置
								boats[i].goods_loaded = 0;//已经决定发出go信号了，更新已经载货数为0
							}
						}
						else
						{//泊位上数量大于等于船的剩余容量，说明装满了，可以go
							boats[i].go_number = 1;
							berths[boats[i].berthId].count = berths[boats[i].berthId].count - (Capacity - boats[i].goods_loaded);//更新泊位数量
							boats[i].goods_loaded = 0;//决定要go了，将携带货物数置零
							berths[boats[i].berthId].ship_exist = 0;//泊位标为闲置
						}
					}
				}
				else
				{//如果上个状态是0或2，此时状态是1，说明此时上个循环-这个循环的过程中，或恰好是这个循环船到了，
				//暂时假设是当前循环恰好到，记录当前帧id到船，即船到达时的帧id
					boats[i].arrive_id = FrameId;
					boats[i].go_number = 0;
				}
			}

			else
			{//没在泊位
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
			{//遍历每艘船
				if (FrameId>=(15000 - berths[boats[i].berthId].transport_time - 10))
				{//再不run就来不及了
					boats[i].ship[0] = 0;
					boats[i].go_number = 1;
				}
			}
		}
	}
	//************************************************************************************************************

};

int main() {
	Controller controller; // 创建控制器对象
	controller.MapsRead(); // 读取地图信息
	controller.berth_boatCapcity_read(); // 读取泊位和船只容量信息
	controller.maxtime_init();//初始化maxtime
	for (int zhen = 1; zhen <= 15000; zhen++) // 循环进行控制
	{/*读取、存储数据*/
		controller.frameId_money_read(); // 读取帧ID和金钱信息
		controller.goodsRead(); // 读取货物信息
		controller.robotRead(); // 读取机器人信息
		controller.boatRead(); // 读取船只信息
		/*中间过程*/
		controller._robot_assign();//给机器人分配目标位置
		controller.ship_number();//给闲置的船分配目标泊位
		controller.go_number();//给船发出go信号
		controller.retreat();//判断要不要全军撤退
		controller.laststate_record();//记录船的当前循环状态作为下个循环的laststate
		//给船分配目标泊位
/*输出*/

	//机器人指令
		controller._output_move();
		controller._output_get();
		controller._output_pull();   //新函数指令
		//船指令
		controller.output_ship();
		controller.output_go();

		puts("OK"); // 输出OK
		fflush(stdout); // 清空输出缓冲区
	}
	return 0; // 返回0，表示程序正常结束
}
