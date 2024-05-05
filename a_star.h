#include <bits/stdc++.h>
#define loca pair<int,int>
#define wall '#'
#define ocean '*'
#define robot 'A'
#define road '.'

const int map_size=205;
using namespace std;
//loca类型运算符重载
loca operator -(loca & a,loca &b){
        return {a.first-b.first,a.second-b.second};
   }

struct node{

    loca l;
    vector <shared_ptr<node>> neighbors;
    shared_ptr<node> father_;
    node(loca L){
        l=L;
        g_cost=0;
        f_cost=0;
        h_cost=0;
        father_=nullptr;
    }
    int g_cost;
    int f_cost;
    int h_cost;
    //计算移动代价
    void set_move_cost(){
        if(father_ == nullptr){
        g_cost = 0;
        return;
    }
    g_cost = father_->g_cost+1;
    }
   //找本节点的邻居
   void change_cost(int new_g){
    g_cost=new_g;
     f_cost=h_cost+new_g;
   }

   
};
typedef map<loca,shared_ptr<node>> mpp;
struct cmp{
    bool operator ()(const shared_ptr<node>a,const shared_ptr<node>b){
          if(a->f_cost>b->f_cost) return true;
          else if(a->f_cost==b->f_cost&&a->h_cost>b->h_cost){
                return true;
          }
          else return false;
    }
    };

struct frontier{
    priority_queue<shared_ptr<node>,vector<shared_ptr<node>>,cmp> searching;
    mpp cost_so_far;
    void put(shared_ptr<node>s){
        searching.emplace(s);
        cost_so_far[s->l]=s;
    }
    shared_ptr<node> pop(){
       shared_ptr<node> out=searching.top();
       //mpp::const_iterator pr;
       /*do{out=searching.top();searching.pop();}
       while(pr =cost_so_far.find(out->l)==cost_so_far.end());*/
       // out=pr->second;     
                                                                     
        cost_so_far.erase(out->l);
        searching.pop();
        return out;
    }
};

struct a_star{
    
    frontier openlist;
    mpp closed;
    loca _start;
    loca goal; 
    list<int> find_path;//存储机器人命令的栈
    a_star(loca start_ ={0,0},loca goal_={0,0}){
        _start =start_;
        goal=goal_;

    };
    void cauculte_cost_add(shared_ptr<node> now,loca &goal){
        now->h_cost=abs(goal.first- now->l.first)+abs(goal.second-now->l.second);
        now->set_move_cost();
        now->f_cost=now->g_cost+now->h_cost;
    };
    
    void setneighbors(char (&map)[map_size][map_size],shared_ptr<node> n ){
        vector<loca> L{{n->l.first,n->l.second-1},{n->l.first,n->l.second+1},{n->l.first-1,n->l.second},{n->l.first+1,n->l.second}};
    

        if(L[0].second>-1&& map[L[0].first][L[0].second]!=wall &&map[L[0].first][L[0].second]!=ocean&&map[L[0].first][L[0].second]!=robot&& closed.find(L[0])==closed.end()){
            n->neighbors.push_back(shared_ptr<node> (new node({L[0].first,L[0].second})));
        }
        if(L[1].second<200&&map[L[1].first][n->l.second+1]!=wall &&map[L[1].first][L[1].second]!=ocean&&map[L[1].first][L[1].second]!=robot&& closed.find(L[1])==closed.end()){

            n->neighbors.push_back(shared_ptr<node> (new node({L[1].first,L[1].second})));

        }
        if(L[2].first>-1&&map[L[2].first][L[2].second]!=wall &&map[L[2].first][L[2].second]!=ocean&&map[L[2].first][L[2].second]!=robot&& closed.find(L[2])==closed.end()){
        n->neighbors.push_back(shared_ptr<node> (new node({L[2].first,L[2].second})));

        }
        if(L[3].first<200&&map[L[3].first][L[3].second]!=wall &&map[L[3].first][L[3].second]!=ocean&&map[L[3].first][L[3].second]!=robot&& closed.find(L[3])==closed.end()){
            n->neighbors.push_back(shared_ptr<node> (new node({L[3].first,L[3].second})));

        }
    }
    list<int> a_star_search(char (&map)[map_size][map_size]){
        if (_start==goal){
            find_path.push_front(-1);
            return find_path;

        }
            shared_ptr<node> start(new node(_start));
            cauculte_cost_add(start,goal);
            openlist.put(start);
    while(openlist.searching.size()>0){
        
        shared_ptr<node> current=openlist.pop();
        if(current->l==goal) {closed.emplace(current->l,current);break;}//终点加入了
        setneighbors(map,current);
        for(shared_ptr<node> now:current->neighbors){
    
            int g_current_cost=current->g_cost+1;
            mpp::const_iterator pr=openlist.cost_so_far.find(now->l);
            if(pr!=openlist.cost_so_far.end()){
                if(pr->second->g_cost>g_current_cost){
                 pr->second->father_=current;
                 pr->second->change_cost(g_current_cost);}
            }
            else{
                now->father_=current;
                cauculte_cost_add(now,goal);
                openlist.put(now);
            }
        }
        closed.emplace(current->l,current);
       
    }
       construct_path();
       return  find_path;


     }

     //构建机器人移动命令的函数
    void construct_path(){
        shared_ptr<node> go_node;//移动节点
        map<loca,int> instruct={{{1,0},2},{{-1,0},3},{{0,1},1},{{0,-1},0}};//存储命令的查找字典
        mpp::const_iterator pr=closed.find(goal); //终点是否在
        if(pr!=closed.end()){
            shared_ptr<node> end_node =pr->second;
            while(end_node->father_!=nullptr){
                go_node =end_node->father_;
                loca d =go_node->l-end_node->l;
                find_path.push_front(instruct.find(d)->second);
                end_node=go_node;   }
        }
        else find_path.push_front(-1);
        }

};