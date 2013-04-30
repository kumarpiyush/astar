#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <map>
#include <set>
#include <utility>
#include <climits>
#include <cfloat>
#include <cassert>
using namespace std;


template <typename state>
class AStar{
    typedef vector<state> path;
    typedef set<state> stateset;
    typename set<state>::iterator itr;

    public:
    AStar(){}
    path run_astar(state start, state goal,int& opensetsize, int& closedsetsize){
        map<state,int> g_score;//(neighbors.size(),0);
        map<state,int> f_score;//(neighbors.size(),0);

        stateset openset,closedset;
        openset.insert(start);

        g_score[start]=0;
        f_score[start]=start.heuristic();

        map<state,state> caller;                // parent "pointer" keeper, to trace back the correct path
        caller[start]=start;                    // sentry value, assuming no self-loops in neighbor vector

        while(openset.size()>0){
#ifdef db
            printf("loop: %d %d\n",(int)openset.size(),(int)closedset.size());
#endif
            // get the least cost node
            state curr;
            int minscore=INT_MAX;
            for (itr=openset.begin();itr!=openset.end();itr++){
                if(f_score[(*itr)]<minscore){
                    curr=(*itr);
                    minscore=f_score[(*itr)];
                }
            }
            if(curr==goal){
                path ret;
                while(!(curr==caller[curr])){
                    ret.push_back(curr);
                    curr=caller[curr];
                }
                ret.push_back(start);
                opensetsize=openset.size();
                closedsetsize=closedset.size();
                return ret;
            }

            openset.erase(curr);
            closedset.insert(curr);

            // update all the neighbors

            vector<state> neighbors=curr.neighbor_vec();
            for(int i=0;i<neighbors.size();i++){
                state neigh=neighbors[i];
                if(closedset.find(neigh)!=closedset.end()){ // alredy in closed set, still check for non monotonicity
                    int tmp_g_score=g_score[curr]+curr.dist(neigh);
                    if(tmp_g_score<g_score[neigh]){
#ifdef db
                        cout<<"parent pointer change for\n";
                        neigh.print_node();
                        cout<<endl;
#endif
                        // do parent pointer redirection here
                        caller[neigh]=curr;
                        openset.insert(neigh);
                        closedset.erase(neigh);
                        g_score[neigh]=tmp_g_score;
                    }
                    else{
                        // already good
                    }
                    continue;
                }

                int tmp_g_score=g_score[curr]+curr.dist(neigh);
                if(openset.find(neigh)==openset.end() or tmp_g_score<g_score[neigh]){
                    g_score[neigh]=tmp_g_score;
                    f_score[neigh]=g_score[neigh]+neigh.heuristic();
                    caller[neigh]=curr;
                    if(openset.find(neigh)==openset.end()){
                        openset.insert(neigh);
                    }
                }
            }
        }
        // not found
        path null;
        return null;
    }
};



int mode=0;

// Implementation for eight puzzle
class epp{
    public:
    int mat[3][3];  // empty block is 0
    epp(){}
    epp(int** puz){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                mat[i][j]=puz[i][j];
            }
        }
    }

    epp(const epp& p2){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                mat[i][j]=p2.mat[i][j];
            }
        }
    }

    int dist(epp i2){
        int ret=0;
        if(mode==0){        // manhattan
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    if(mat[i][j]==0) continue;
                    for(int i1=0;i1<3;i1++){
                        for(int j1=0;j1<3;j1++){
                            if(mat[i][j]==i2.mat[i1][j1]){
                                ret+=abs(i-i1)+abs(j-j1);
                            }
                        }
                    }
                }
            }
        }
        else if(mode==1){   // displacement
            ret=1;
        }
        return ret;
    }

    int heuristic(){
        if(mode==0){
            epp goal;
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    goal.mat[i][j]=3*i+j+1;
                }
            }
            return (this->dist(goal));
        }
        else if(mode==1){
            int ret=0;
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    if(mat[i][j]!=0 and mat[i][j]!=3*i+j+1) ret++;
                }
            }
            return ret;
        }
    }
    vector<epp> neighbor_vec(){
        vector<epp> ret;
        int x,y;
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                if(mat[i][j]==0){
                    x=i;
                    y=j;
                }
            }
        }
        if(x>0){
            epp tmp(*this);
            swap(tmp.mat[x][y],tmp.mat[x-1][y]);
            ret.push_back(tmp);
        }
        if(y>0){
            epp tmp(*this);
            swap(tmp.mat[x][y],tmp.mat[x][y-1]);
            ret.push_back(tmp);
        }
        if(x<2){
            epp tmp(*this);
            swap(tmp.mat[x][y],tmp.mat[x+1][y]);
            ret.push_back(tmp);
        }
        if(y<2){
            epp tmp(*this);
            swap(tmp.mat[x][y],tmp.mat[x][y+1]);
            ret.push_back(tmp);
        }
        return ret;
    }

    bool operator==(epp i2)const{
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                if(mat[i][j]!=i2.mat[i][j]) return false;
            }
        }
        return true;
    }
    bool operator>(epp i2)const{
        return this->to_string()>i2.to_string();
    }
    bool operator<(epp i2)const{
        return this->to_string()<i2.to_string();
    }

    string to_string()const{
        string str;
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                str+=(mat[i][j]+'0');
            }
        }
        return str;
    }
    void print_node(){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                cout<<mat[i][j]<<" ";
            }
            cout<<"\n";
        }
    }
};

void run_for_epp(){
    AStar<epp> as;
    epp goal;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            goal.mat[i][j]=3*i+j+1;
        }
    }
    goal.mat[2][2]=0;
    epp start;
    cin>>start.mat[0][0];
    cin>>start.mat[0][1];
    cin>>start.mat[0][2];
    cin>>start.mat[1][0];
    cin>>start.mat[1][1];
    cin>>start.mat[1][2];
    cin>>start.mat[2][0];
    cin>>start.mat[2][1];
    cin>>start.mat[2][2];
    cout<<"Enter mode\nmanhattan:0\ndisplaced count:1\n>>> ";
    cin>>mode;
    if(mode<0 or mode>1){
        cout<<"wrong mode!\n";
        exit(1);
    }

    printf("now running\n");


    int open,closed;
    vector<epp> p=as.run_astar(start,goal,open,closed);

    // now verify that solution indeed satisfies move constraints
    for(int i=p.size()-1;i>=0;i--){
        p[i].print_node();
        cout<<"\n";
    }
    printf("Solution path size %d\n",(int)p.size());
    printf("Nodes count expanded:: Open %d, Closed %d\n",open,closed);
}


int main(){
    srand(time(0));
    run_for_epp();
    return 0;
}