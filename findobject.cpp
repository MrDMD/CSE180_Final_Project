#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

//Global variables for data including laser scan, and odometry
geometry_msgs::PoseWithCovarianceStamped curr_post;
sensor_msgs::LaserScan laser;
std_msgs::Bool laser_ready;

const double Inc = 0.0065540750511;
const int Z = 100;
const int range = 50;
bool uu = true;
std::string ttt = "/home/daniel/Documents/cse-180/CSE180/src/assignments/src/patterns/table.txt";
std::string mmm = "/home/daniel/Documents/cse-180/CSE180/src/assignments/src/patterns/mailbox.txt";
std::string ddd = "/home/daniel/Documents/cse-180/CSE180/src/assignments/src/patterns/dirty.txt";

float getXY(float var){

    //curr_post.pose.position();

    return var; 
}

enum OBJECTS{
    table,
    mailbox,
};

int Fl(float c){ return (int)c; }

struct Point{

    int x;
    int y;
    int pat;
    OBJECTS obj;

};



class Lpat{
    public:
    OBJECTS type;
    int size;
    int patterns;
    std::string filename;
    std::vector < std::vector < std::vector < std::vector < int > > > > laser_map;

    Lpat(){}

    Lpat(std::string filename){
        
        this->filename = filename;

        std::ifstream file(filename);
        
        if (!file)
        {
            std::cerr << "Could not open file!"  << std::endl;
        }
        else
        {
        std::cout << "File Opened" << std::endl;
        float c;
        file >> c; this->patterns = Fl(c);
        if(c == 0){
            type = table;
            std::cout << "Obect Type Table\n";
        }else if(c == 1){

            type = mailbox;
            std::cout << "Obect Type Mailbox\n";
            
        }else{

            std::cerr << "Could not determine type of object for pattern!"  << std::endl;

        }
        //get number of patterns
        file >> c; this->patterns = Fl(c);
        std::cout << "Patterns in file: " << patterns << std::endl;
        //patterns;
        //get the size of the nxn matrix of the pattern
        file >> c; this->size =  Fl(c);;
        std::cout << "Size of matrix: " << size << std::endl;
        
        //resize vectors to fit patterns'
        laser_map.resize(4);
    

        for(int i = 0; i < 4; i++)
        laser_map[i].resize(patterns);


        std::vector < std::vector < int > > matrix(size);

        for ( int i = 0 ; i < size ; i++ )
        matrix[i].resize(size, 0);

        for (int i = 0; i < patterns && file.good(); ++i) 
        {
                
            for(int v = 0; v < 4; v++)
            laser_map[v][i] = matrix;

            

            for(int j = 0; j < size && file.good(); j++)
            {
            
                for(int k = 0; k < size && file.good(); k++)
                {
                    for(int b = 0; b < 4; b++){
                        if(b == 0){
                            
                            file >> c; laser_map[b][i][j][k] =  Fl(c);
                            std::cout << laser_map[b][i][j][k] << " ";
                        
                        }else if(b==1){

                            laser_map[b][i][(size - j)-1][(size - k)-1] = laser_map[0][i][j][k];

                        }else if(b==2){

                            laser_map[b][i][j][(size - k)-1] = laser_map[0][i][j][k];

                        }else if(b==3){

                            laser_map[b][i][(size-j-1)][k] = laser_map[0][i][j][k];

                        }
                    }

                }
                std::cout << std::endl;
            }
            //Reads Left to right, top to bottom.
            std::cout << "\n\n";
        }

        }
    }

    
    int GetSize(){
        return size;
    }

    int GetPatterns(){
        return size;
    }
 
};

class Obj{
public:        
    OBJECTS type;
    Lpat patterns;
    Point on_map;

    Obj(){
    }

    Obj(std::string str, OBJECTS obT){
        patterns = *new Lpat(str);
        this->type = obT;
    }
    

    void on_map_set(int x, int y){

        on_map.x = x;

        on_map.y = y;

    }

    float distance(int x1, int y1, int x2, int y2) 
    { 
        // Calculating distance 
        return sqrt(pow(x2 - x1, 2) +  
                    pow(y2 - y1, 2) * 1.0); 
    }

    float demap(float num){
        num = (num - range);
        return num/10;
    }

    std::string sst(OBJECTS type){
        if(type==0)
        return "table";

        if(type==1)
        return "mailbox";
    }

    void on_map_find(std::vector<Point> matches, int m){
        int k = 0;
        Point p;
    
        //write logic for determinit there is a table
        if(type == table){
            //a table should have at least 2 legs found in order to affirm we found a table
            for(int i = 0; i < (m-1); i+=2){
                    if(matches[i].obj == table){ 

                        if((distance(matches[i].x, matches[i].y, matches[i+1].x, matches[i+1].y))/10 != 0 && 
                           (distance(matches[i].x, matches[i].y, matches[i+1].x, matches[i+1].y))/10 <= 2.5 && 
                           (distance(matches[i].x, matches[i].y, matches[i+1].x, matches[i+1].y))/10 >= 1.0 )
                        {

                        k++;
                        std::cout << "obect at: " << matches[i].x <<","<< matches[i].y << " is a leg with object at:" << matches[i+1].x <<","<< matches[1+1].y <<std::endl;
                        
                        p.x = matches[i].x;
                        p.y = matches[i].y; 

                        }
                    }

            }

            if(k>0){
            std::cout << RED << curr_post.pose.pose.position.x << "," << curr_post.pose.pose.position.y   << "\n" <<demap(p.x) << ","  << demap(p.y) << RESET <<std::endl;
            }

        }
        //determine if there is a mailbox
        else if(type == mailbox){

            for(int i = 0; i < m; i++){
                for(int j = 0; j < m; j++){
                    if(matches[i].obj == matches[j].obj && matches[i].obj == mailbox && matches[j].obj == mailbox){ 
                        std::cout << RED << curr_post.pose.pose.position.x + demap(matches[i].y)<< "," 
                        << curr_post.pose.pose.position.y + demap(matches[i].x) << RESET <<std::endl;
                    }
                }
            }

        }

    }

};

class Lmap{
public:
    Obj Tt;
    Obj Mm;
    int size;
    std::vector< std::vector<int> > laser_map;

    Lmap(const int z){
        OBJECTS type1 = table;
        OBJECTS type = mailbox;
        Tt = *new Obj(ttt, type1);
        Mm = *new Obj(mmm ,type);
        size = z;
        laser_map.resize(size,std::vector<int>(size,0));

    }
    
    int GetSize(){
        return size;
    }

    float remap(float num){
        if( num >= -5 && num <= 5){
            num = (num*10)+range; 
            return num;
        }else{
            return (-1);
        }
    }

    float demap(float num){
        return (num/10)-range;
    }

    //translates what the laser is getting into a simple 2d bit map
    void PtC (float angle, float r){
        //translates to cartesian
        float x, y;
        x = r*cos(angle);
        y = r*sin(angle);

        if(remap(x)!=-1 && remap(y)!=-1){
            laser_map[int(remap(x))][int(remap(y))] = 1;
        }
    }

    char pt(int i){
        char c;

        if(i==0){
            c = ' ';
        }else{
            c = '*';
        }

        return c;
    }

    //function for debbuging
    void printl(std::vector<Point> matches, int m){

        for(int i = 0; i < size; i++){

            for(int j = 0; j < size; j++){

                bool h = false;
                for(int k=0; k < m; k++){
                    if(matches[k].x == i && matches[k].y == j){
                        h = true;
                    }
                }

                if(h){
                std::cout << RED << pt(laser_map[i][j]) << '|' << RESET;
                }else{
                std::cout << pt(laser_map[i][j]) << '|' ;
                }

            }

            std::cout << std::endl;
        
        }

    }


   
    void savepattern(std::vector<Point> matches, int m){

        //remember to comment this out
            while(uu){

            int x=0, y=0;
            int flag;

            std::cout << "\nDefine x\n"; 
            std::cin >> y; 
            std::cout << "\nDefine y\n"; 
            std::cin >> x;

            if(x == -1)
            uu = false;

            for(int i = 0; i < size; i++){

                for(int j = 0; j < size; j++){

                    bool h = false;
                    for(int k=0; k < m; k++){
                        if(matches[k].x == i && matches[k].y == j){
                            h = true;
                        }
                    }

                    if(i >= x && i <= x+19 && j >= y && j <= y+19){
                    if(h){
                    std::cout << RED << pt(laser_map[i][j]) << '|' << RESET;
                    }else{
                    std::cout << pt(laser_map[i][j]) << '|' ;
    
                    }
                    }
                    
                }

                if(i >= x && i <= x+19){
                std::cout << std::endl;
                }
            }

            std::fstream myfile;
            myfile.open(ddd, std::fstream::app);

            if (myfile.is_open()) 
            {
                myfile<< std::endl;
                std::cout << "pattern saver open file\n";
                std::cout << "Would you like to save this pattern?\n";
                std::cin >> flag;

                if(flag!=-1){
                    for(int i = 0; i < size; i++){

                        for(int j = 0; j < size; j++){
                            
                            bool h = false;
                            
                            for(int k=0; k < m; k++){
                                if(matches[k].x == i && matches[k].y == j){
                                    h = true;
                                }
                            }

                            if(i >= x && i <= x+19 && j >= y && j <= y+19){
                             myfile << laser_map[i][j] << " ";
                            }

                           

                        }
                        
                        if(i >= x && i <= x+19){
                            myfile<< std::endl;
                        }
                    }

                }

                myfile.close();

            }
            else 
            {
                std::cout << "Cannot open file";
            }
            }
        }

    
    
    void clean(){
        laser_map.clear();
        laser_map.resize(size,std::vector<int>(size,0));
    }

    void search(){
        for(int bit = 0; bit < 2; bit++){

            search_one(bit);

        }
    }


    private:
    void search_one(int bit){
        Obj some;
        if(bit == 0){ some = Tt; }else if(bit == 1){ some = Mm; }
    
        int S = size;
        int M = some.patterns.size;

        int flag, i, j, p, q, l, m;
        std::vector<Point> matches (200);
        m = 0;

        for(int a = 0; a < 4; a++){

            for(l = 0; l < some.patterns.patterns; l++){
                for(i=0; i<=(S-M); i++)
                {
                    for(j=0; j<=(S-M); j++)
                    {
                        flag=0;
                        for(p=0; p<M; p++)
                        {
                            for(int q=0; q<M; q++)
                            {
                                if(laser_map[i+p][j+q] != some.patterns.laser_map[a][l][p][q])
                                {
                                flag=1;
                                break;
                                }

                            }
                        }
                        if(flag==0)
                        {
                            bool dupi = false;
                            for(int dup = 0; dup < m; dup++){
                                if(matches[dup].x == (i+1) && matches[dup].y == (j+1) && matches[dup].pat == l){
                                    dupi = true;
                                }
                            }
                            if(!dupi){
                            matches[m].x = (i+1);  matches[m].y = (j+1); matches[m].pat = l; matches[m].obj = some.type;
                            m++;
                            std::cout << "Match Found in the Main Matrix at starting location " << (i+1) << "," << (j+1) <<" for pattern " << l << " Object type: " << some.sst(some.type) <<std::endl;
                            }       
                        }   
                    }
                    if(flag==0)
                    {
                        bool dupi = false;
                            for(int dup = 0; dup < m; dup++){
                                if(matches[dup].x == (i+1) && matches[dup].y == (j+1) && matches[dup].pat == l){
                                    dupi = true;
                                }
                            }
                            if(!dupi){
                            matches[m].x = (i+1);  matches[m].y = (j+1); matches[m].pat = l; matches[m].obj = some.type;
                            m++;
                            std::cout << "Match Found in the Main Matrix at starting location " << (i+1) << "," << (j+1) <<" for pattern " << l << " Object type: " << some.sst(some.type) <<std::endl;
                            }       
                    }
                }
            }

        }


        some.on_map_find(matches, m);
        printl(matches, m);
        //savepattern(matches, m);

    }


};

void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser_ready.data=true;
    laser = *msg;
    return;
}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
	curr_post = *msg;
	return;
}	

int main(int argc, char** argv) {
    ros::init(argc, argv, "findobject");
    ros::NodeHandle nh;

    //Node suscribers laser, odom, target position;
    ros::Subscriber laser_sub = nh.subscribe("/scan", 1000, &scanCB);

    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &amclCB);

    //laser data takes a little to kick in and trows core dumped;
    laser_ready.data=false;

    //process pattern to be used for object detection
    Lmap lasermap(Z);

    ros::Rate rate(0.5);
    while(ros::ok()){
        ros::spinOnce();

        if(!laser.ranges.empty()) {
            float inc = 2.3561899662;
        for (int i = 0; i < 720; i++)
            {   
                if(i!=0)
                inc = inc - Inc;
                //lasermap.laser_map.clear();
                lasermap.PtC(inc, laser.ranges[i]);
            }
            ROS_WARN_STREAM("\n\n");
            lasermap.search();
            uu=true;
            lasermap.clean();
        }
        rate.sleep();
    }
}
