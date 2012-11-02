//
//  astar.cpp
//  561assign2
//
//  Created by Aditya Raghuwanshi on 10/10/12.
//
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <sstream>
#include <fstream>

using namespace std;

int n; // horizontal size of the map
int m; // vertical size size of the map
int nodesExpanded=0;
static int map[100][100];
static int closed_nodes_map[100][100]; // map of closed (tried-out) nodes
static int open_nodes_map[100][100]; // map of open (not-yet-tried) nodes
static int dir_map[100][100]; // map of directions
const int dir=4; // number of possible directions to go at any position
// if dir==4
static int dx[dir]={1, 0, -1, 0};
static int dy[dir]={0, 1, 0, -1};
static int dBx[dir]={0, 1, 0, -1};
static int dBy[dir]={-1, 0, 1, 0};
// if dir==8
//static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
//static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

struct results
{
    int nodesExpanded;
    string route;
    int type;
};

class node
{
    // current position
    int xPos;
    int yPos;
    //current B position
    int BxPos;
    int ByPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority
    
public:
    node(int xp, int yp, int d, int p, int Bxp, int Byp) 
    {xPos=xp; yPos=yp; level=d; priority=p; BxPos=Bxp; ByPos=Byp;}
    
    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    int getBxPos() const {return BxPos;}
    int getByPos() const {return ByPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}
    
    void updatePriority(const int & xDest, const int & yDest, int type)
    {
        priority=level+estimate(xDest, yDest, type)*10; //A*
    }
    
    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i) // i: direction
    {
        level+=(dir==8?(i%2==0?10:14):10);
    }
    
    // Estimation function for the remaining distance to the goal.
    const int & estimate(const int & xDest, const int & yDest, int type) const
    {
        static int xd, yd, d;
        xd=xDest-xPos;
        yd=yDest-yPos;         
        
        if(type==1) {
            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));
        }
        
        if(type==2) {
            // Manhattan distance
            d=abs(xd)+abs(yd);
        }
        
        if(type==3) {
            // Chebyshev distance
            d=max(abs(xd), abs(yd));
        }
        
        return(d);
    }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart, 
                const int & xFinish, const int & yFinish, int type )
{
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy, Bx, By, Bxdx, Bydy;
    static char c;
    pqi=0;
    
    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }
    
    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0, xFinish, yFinish);
    n0->updatePriority(xFinish, yFinish, type);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map
    
    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
                    pq[pqi].top().getLevel(), pq[pqi].top().getPriority(),
                    pq[pqi].top().getBxPos(), pq[pqi].top().getByPos());
        
        x=n0->getxPos(); y=n0->getyPos();
        Bx=n0->getBxPos(); By=n0->getByPos();
        
        //printf("Examining A:%d,%d & B:%d,%d\n",x,y,Bx,By);
        nodesExpanded++;
        
        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;
        
        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==Bx && y==By) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }
            
            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
        }
        
        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];
            Bxdx=Bx+dBx[i]; Bydy=By+dBy[i];
            
            //printf("Trying A:%d,%d & B:%d,%d\n",xdx,ydy,Bxdx,Bydy);
            
            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1 
                 || closed_nodes_map[xdx][ydy]==1 || Bxdx<0 || Bxdx>n-1 
                 || Bydy<0 || Bydy>m-1 || map[Bxdx][Bydy]==1))
            {
                //printf("Moved\n");
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), 
                            n0->getPriority(), Bxdx, Bydy);
                m0->nextLevel(i);
                m0->updatePriority(Bx, By, type);
                
                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                    //printf("Added to Open List\n");
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    //printf("Added as better node\n");
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                    
                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx && 
                            pq[pqi].top().getyPos()==ydy))
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted node
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

int main()
{
    string line,route;
    ifstream myfile ("input.txt");
    int sA[2],sB[2],obstacle[2];
    int xA, yA, xB, yB;
    int i;
    int input=1;
    string output1,output2;
    char buff[1024];
    char buff2[1024];
    results result;
    
    sprintf(buff,"\t\tNodes Expanded\n\tEuclidean\tManhattan\tChessboard\n");
    output1.append(buff);
    
    sprintf(buff2,"Optimal Path Solution:\n");
    output2.append(buff2);
    
    if(!myfile.is_open()) {
        printf("Error opening input file\n");
        return(1);
    }
    while (myfile.good()) { 
        sprintf(buff,"Input%d\t",input);
        output1.append(buff);
        
        sprintf(buff2,"Input%d: ",input++);
        output2.append(buff2);
        
        result.nodesExpanded=0;
        result.route="";
        result.type=0;
        i=0;
        //get size
        getline (myfile,line);
        n=m=atoi(line.c_str());
        //get startA
        getline (myfile,line);
        stringstream streamA(line);
        string startA;
        while( getline(streamA, startA, ' ') ) {
            sA[i++]=atoi(startA.c_str());
        }
        i=0;
        //get startB
        getline (myfile,line);
        stringstream streamB(line);
        string startB;
        while( getline(streamB, startB, ' ') ) {
            sB[i++]=atoi(startB.c_str());
        }
        xA=sA[1]-1;
        yA=sA[0]-1;
        xB=sB[1]-1;
        yB=sB[0]-1;
        
        // create empty map
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++) map[x][y]=0;
        }
        
        //get obstacles
        while ( myfile.good() ) {
            getline (myfile,line);
            if(line.empty()) {
                break;
            }
            stringstream streamO(line);
            string obStr;
            i=0;
            while( getline(streamO, obStr, ' ') ) {
                obstacle[i++]=atoi(obStr.c_str());
            }
            map[obstacle[1]-1][obstacle[0]-1]=1;
        }
        
        nodesExpanded=0;
        //Eucledian
        //cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
        //cout<<"Start A: "<<xA<<","<<yA<<endl;
        //cout<<"Start B: "<<xB<<","<<yB<<endl;
        // get the route
        route=pathFind(xA, yA, xB, yB,1);
        //if(route=="") cout<<"An empty route generated!"<<endl;
        //cout<<"Route:"<<endl;
        //cout<<route<<endl<<endl;
        // follow the route on the map and display it 
        if(route.length()>0)
        {
            /*
            int j; char c;
            int x=xA;
            int y=yA;
            map[x][y]=2;
            for(int i=0;i<route.length();i++)
            {
                c =route.at(i);
                j=atoi(&c); 
                x=x+dx[j];
                y=y+dy[j];
                map[x][y]=3;
            }
            map[x][y]=4;
            
            // display the map with the route
            for(int y=0;y<m;y++)
            {
                for(int x=0;x<n;x++)
                    if(map[x][y]==0)
                        cout<<".";
                    else if(map[x][y]==1)
                        cout<<"O"; //obstacle
                    else if(map[x][y]==2)
                        cout<<"S"; //start
                    else if(map[x][y]==3)
                        cout<<"R"; //route
                    else if(map[x][y]==4)
                        cout<<"F"; //finish
                cout<<endl;
            }
             */
            result.nodesExpanded=nodesExpanded-1;
            result.route=route;
            result.type=1;
        }
        else {
            nodesExpanded=0;
        }
        if(nodesExpanded==0) {
            sprintf(buff,"\t-");
            output1.append(buff);
        } else {
            sprintf(buff,"\t%d",nodesExpanded-1);
            output1.append(buff);
        }
        
        //printf("Nodes Expanded: %d\n",--nodesExpanded);
        nodesExpanded=0;
        
        //Manhattan
        
        route=pathFind(xA, yA, xB, yB,2);
        //if(route=="") cout<<"An empty route generated!"<<endl;
        //cout<<"Route:"<<endl;
        //cout<<route<<endl<<endl;
        // follow the route on the map and display it 
        if(route.length()>0)
        {
            if(result.nodesExpanded>(nodesExpanded-1)) {
                result.nodesExpanded=nodesExpanded-1;
                result.route=route;
                result.type=2;
            }
        }
        else {
            nodesExpanded=0;
        }
        if(nodesExpanded==0) {
            sprintf(buff,"\t\t-");
            output1.append(buff);
        } else {
            sprintf(buff,"\t\t%d",nodesExpanded-1);
            output1.append(buff);
        }
        //printf("Nodes Expanded: %d\n",--nodesExpanded);
        nodesExpanded=0;
        
        //ChessBoard
        route=pathFind(xA, yA, xB, yB,3);
        //if(route=="") cout<<"An empty route generated!"<<endl;
        //cout<<"Route:"<<endl;
        //cout<<route<<endl<<endl;
        // follow the route on the map and display it 
        if(route.length()>0)
        {
            if(result.nodesExpanded>(nodesExpanded-1)) {
                result.nodesExpanded=nodesExpanded-1;
                result.route=route;
                result.type=3;
            }
        }
        else {
            nodesExpanded=0;
        }
        if(nodesExpanded==0) {
            sprintf(buff,"\t\t-\n");
            output1.append(buff);
        } else {
            sprintf(buff,"\t\t%d\n",nodesExpanded-1);
            output1.append(buff);
        }
        
        //printf("Nodes Expanded: %d\n",--nodesExpanded);
        nodesExpanded=0;
        //printf("Winning Result = %d, %s, %d\n",result.nodesExpanded,result.route.c_str(),result.type);
        if(result.nodesExpanded!=0) {
            int j; char c;
            int x=xA;
            int y=yA;
            for(int i=0;i<result.route.length();i++)
            {
                c =route.at(i);
                j=atoi(&c); 
                x=x+dx[j];
                y=y+dy[j];
                sprintf(buff2," (%d,%d) ",y+1,x+1);
                output2.append(buff2);
            }
        } else {
            sprintf(buff2," -1 ");
            output2.append(buff2);
        }
        sprintf(buff2,"\n");
        output2.append(buff2);
    }
    myfile.close();
    
    sprintf(buff,"\n");
    output1.append(buff);

    //printf("%s",output1.c_str());
    //printf("%s",output2.c_str());
    
    ofstream outfile ("output.txt");
    if(!outfile.is_open()) {
        printf("Error opening output file\n");
        return(1);
    }
    outfile<<output1<<output2;
    outfile.close();
    return(0);
}

