#include <math.h>
#include <functional>
#include <queue>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <fstream>
#include <string>
#include <ctime>
#include <tuple>
#include "Astar.cpp"
#include "mex.h"
using namespace std;

#define MAX_CAPACITY 0.2
#define MAP_IN		prhs[0]
#define ROBOT_IN	prhs[1]
#define DEPLETE 	prhs[2]
#define	ACTION_OUT	plhs[0]

class mystream : public std::streambuf
{
protected:
virtual std::streamsize xsputn(const char *s, std::streamsize n) { mexPrintf("%.*s", n, s); return n; }
virtual int overflow(int c=EOF) { if (c != EOF) { mexPrintf("%.1s", &c); } return 1; }
};
class scoped_redirect_cout
{
public:
  scoped_redirect_cout() { old_buf = std::cout.rdbuf(); std::cout.rdbuf(&mout); }
  ~scoped_redirect_cout() { std::cout.rdbuf(old_buf); }
private:
  mystream mout;
  std::streambuf *old_buf;
};
static scoped_redirect_cout mycout_redirect;

class Robot{
private:
	int x;
	int y;
	string carrying_mat_type;
public:
	void update_state(int x, int y){
		this->x = x;
		this->y = y;
	}
	void update_carried(string mat_type){
		this->carrying_mat_type = mat_type;
	}
	int get_x(){
		return this->x;
	}
	int get_y(){
		return this->y;
	}
};

struct bin_node_struct{
public:
	string bin_type;
	string mat_type;
	int x;
	int y;
	double supply_level;
	bin_node_struct(): bin_type(""), mat_type(""),x(0),y(0),supply_level(0){}
};
typedef struct bin_node_struct bin_node;

class Map{
private: 
	int size_x;
	int size_y;
	vector<bin_node*> warehouse_bins;
	vector<bin_node*> supply_bins;
public:
	void set_size(int x, int y){
		this->size_x = x;
		this->size_y = y;
	}
	void add_warehouse_bin(string mat_type, int x, int y){
		bin_node* bin = new bin_node();
		bin->bin_type = "warehouse";
		bin->mat_type = mat_type;
		bin->x = x;
		bin->y = y;
		this->warehouse_bins.push_back(bin);
	}
	void add_supply_bin(string mat_type, int x, int y){
		bin_node* bin = new bin_node();
		bin->bin_type = "supply";
		bin->mat_type = mat_type;
		bin->x = x;
		bin->y = y;
		bin->supply_level = 1;
		this->supply_bins.push_back(bin);
	}
	void update_supply_level(vector<double> depletion){
		if(depletion.size() != this->supply_bins.size()){
			cout << "#DEPLETION NOT EQUAL TO #SUPPLY_BINS\n";
			return;
		}
		for(int i = 0; i < depletion.size(); i++){
			(this->supply_bins[i])->supply_level = (this->supply_bins[i])->supply_level-depletion[i];
		}
		return;
	}
	vector<bin_node*> get_warehouse_bins(){
		return this->warehouse_bins;
	}
	vector<bin_node*> get_supply_bins(){
		return this->supply_bins;
	}
};

struct goal_node_struct{
public:
  bin_node* bin;
  double c; // distance travelled so far
  double h; 
  goal_node_struct* prev;
  goal_node_struct(): bin(NULL), c(0), h(0), prev(NULL){}
};
typedef struct goal_node_struct goal_node;


// END STRUCT
// START COMPARITOR 
struct compare_goal_nodes{
    bool operator()(goal_node* A, goal_node* B){
      return A->h > B->h;
    };
};

vector<bin_node*> get_goal_successors(goal_node* node, Map* map){
	if((node->bin)->bin_type == "robot"){
		return map->get_warehouse_bins();
	} else if ((node->bin)->bin_type == "warehouse"){
		vector<bin_node*> s = map->get_supply_bins();
		vector<bin_node*> ans;
		for(int i = 0 ; i < s.size(); i++){
			if((s[i])->mat_type == (node->bin)->mat_type){
				ans.push_back(s[i]);
			}
		}
		return ans;
	}
}

double compute_goal_heuristic(goal_node* node){
	if((node->bin)->bin_type == "warehouse"){
		cout<< "Warehouse Heuristic: ";
		cout<< node->c/MAX_CAPACITY;
		cout << "\n";
		return node->c/MAX_CAPACITY;
	} else if((node->bin)->bin_type == "supply"){
		double curr_supply_level = ((node->bin)->supply_level);
		cout << "Supply Heuristic: ";
		if(curr_supply_level > (1-MAX_CAPACITY)){
			cout << node->c/(1-curr_supply_level);
			cout << "\n"; 
			return node->c/(1-curr_supply_level);
		} else {
			cout << node->c/(MAX_CAPACITY);
			cout << "\n";
			return node->c/MAX_CAPACITY;
		}
	}
	return 0;
}
void print_bin_node(bin_node* bin){
	cout << "type: ";
	cout << bin->bin_type;
	cout << "    material ";
	cout << bin->mat_type;
	cout << "    coords: (";
	cout << bin->x;
	cout << ", ";
	cout<< bin->y;
	cout << ")";
	if(bin->bin_type == "supply"){
		cout << "    supply level: ";
		cout << bin->supply_level;
	}
	cout << "\n";
}
void print_goal_node(goal_node* goal){
	print_bin_node(goal->bin);
	cout<< "Distance Travelled: ";
	cout<< goal->c;
	cout<< "    Heuristic: ";
	cout<< goal->h;
}

vector<goal_node*> goal_planner(Map* map, Robot* r){
	cout << "Create Priority Queue\n";
	priority_queue<goal_node*, vector<goal_node*>,compare_goal_nodes> open;
	cout << "Create start node\n";
	goal_node* start = new goal_node();
	bin_node* bin = new bin_node();
	bin->x = r->get_x();
	bin->y = r->get_y();
	bin->bin_type = "robot";
	bin->mat_type = "";
	start->bin = bin;
	start->h = 0;
	start->c = 0;
	start->prev = NULL;
	cout << "Push into Priority Queue\n";
	open.push(start);
	goal_node* top_node;
	cout << "Start loop\n";
	while(open.size()!=0){
		top_node = open.top(); open.pop();
		if((top_node->bin)->bin_type == "supply"){
			break;
		}
		vector<bin_node*> curr_successors = get_goal_successors(top_node, map);
		for(int i = 0 ; i < curr_successors.size(); i++){
			goal_node* curr_node = new goal_node;
			curr_node->bin = curr_successors[i];
			int curr_x = (curr_node->bin)->x;
			int curr_y = (curr_node->bin)->y;
			int last_x = (top_node->bin)->x;
			int last_y = (top_node->bin)->y;
			double dist = sqrt(pow(curr_x-last_x,2) + 
					           pow(curr_y-last_y,2));
			curr_node->c = top_node->c + dist;
			curr_node->h = compute_goal_heuristic(curr_node);
			curr_node->prev = top_node;
			open.push(curr_node);
		}
	}
	vector<goal_node*> ans;
	while(top_node->prev != NULL){
		ans.insert(ans.begin(),top_node);
		top_node = top_node->prev;
	}
	return ans; // returns vector of warehouse and supply
}

void print_plan(vector<goal_node*> goal_plan){
	for(int i = 0 ; i < goal_plan.size(); i++){
		bin_node* bin = goal_plan[i]->bin;
		cout<< "type: ";
		cout<< bin->bin_type;
		cout<< "    material: ";
		cout<< bin->mat_type;
		cout<< "    coords: (";
		cout<< bin->x;
		cout<< ", ";
		cout<< bin->y;
		cout<< ")";;
		if (bin->bin_type == "supply"){
			cout<< "    supply level: ";
			cout<< bin->supply_level;
		}
		cout << "\n";
	}
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[]){
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 

	} /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    /* init map representation */
    Map* m = new Map();
    m->set_size(x_size, y_size);
    for(int i = 0; i < x_size; i++){
    	for(int j = 0; j < y_size; j++){
    		double curr_val = map[j*y_size+i];
    		if(curr_val > 0){
    			m->add_supply_bin(to_string(abs(curr_val)), i, j);
    		} else if (curr_val < 0){
    			m->add_warehouse_bin(to_string(abs(curr_val)), i, j);
    		}
    	}
    }
    y_size = mxGetN(DEPLETE);
    double* deplete = mxGetPr(DEPLETE);
    vector<double>depletion;
    for(int i = 0; i < y_size; i++){
    	depletion.push_back(deplete[i]);
    }
    m->update_supply_level(depletion);
    Robot* r = new Robot();
    double* r_pos = mxGetPr(ROBOT_IN);
    r->update_state(r_pos[0], r_pos[1]);

    vector<goal_node*> goal_plan = goal_planner(m,r);
    plhs[0] = mxCreateDoubleMatrix(1, goal_plan.size()*2, mxREAL);
    double* values = (double*)malloc(sizeof(double)*goal_plan.size()*2);
    for(int i = 0; i < goal_plan.size(); i++){
    	values[i*2] = (goal_plan[i]->bin)->x;
    	values[i*2+1] = (goal_plan[i]->bin)->y;
    }
    memcpy(mxGetPr(plhs[0]), values, sizeof(double)*2*goal_plan.size()*2);
    return;
}




// TEST CODE
// int main(){
// 	cout << "Init Robot\n";
// 	Robot* r = new Robot();
// 	r->update_state(0, 0);
// 	cout << "Init Map\n";
// 	Map* m = new Map();
// 	// m->set_size(8,8);
// 	m->add_warehouse_bin("A", 1, 2);
// 	m->add_warehouse_bin("A", 2, 4);
// 	m->add_warehouse_bin("A", 4, 2);
// 	m->add_warehouse_bin("B", 2, 6);
// 	m->add_warehouse_bin("B", 5, 2);
// 	m->add_warehouse_bin("B", 5, 5);
// 	m->add_supply_bin("A", 0, 0);
// 	m->add_supply_bin("A", 0, 7);
// 	m->add_supply_bin("B", 7, 0);
// 	m->add_supply_bin("B", 7, 7);
// 	vector<double>depletion;
// 	depletion.push_back(0);
// 	depletion.push_back(0.1);
// 	depletion.push_back(0.1);
// 	depletion.push_back(0.2);
// 	m->update_supply_level(depletion);
// 	cout << "Run Goal Planner\n";
// 	vector<goal_node*> goal_plan = goal_planner(m, r);
// 	cout << "Goal Plan Size: ";
// 	cout << goal_plan.size();
// 	cout << "\n";
// 	print_plan(goal_plan);
// 	return 0;

// }