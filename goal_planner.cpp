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
#include "costAStar.cpp"
#include "mex.h"
using namespace std;

#define MAX_CAPACITY 	0.2
#define MAP_IN			prhs[0]
#define ROBOT_IN		prhs[1]
#define MACHINE 		prhs[2]
#define WAREHOUSE  		prhs[3]
#define	X_OUT			plhs[0]
#define Y_OUT			plhs[1]
#define WAYPOINTS_OUT	plhs[2]

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
	vector<bin_node*> machine_bins;
	double** obs;
public:
	void set_size(int x, int y){
		this->size_x = x;
		this->size_y = y;
	}
	void add_warehouse_bin(string mat_type, int x, int y, double supply_level){
		bin_node* bin = new bin_node();
		bin->bin_type = "warehouse";
		bin->mat_type = mat_type;
		bin->x = x;
		bin->y = y;
		bin->supply_level = supply_level;
		this->warehouse_bins.push_back(bin);
	}
	void add_machine_bin(string mat_type, int x, int y, double supply_level){
		bin_node* bin = new bin_node();
		bin->bin_type = "machine";
		bin->mat_type = mat_type;
		bin->x = x;
		bin->y = y;
		bin->supply_level = supply_level;
		this->machine_bins.push_back(bin);
	}
	// void update_supply_level(vector<double> depletion){
	// 	if(depletion.size() != this->machine_bins.size()){
	// 		cout << "#DEPLETION NOT EQUAL TO #MACHINE_BINS\n";
	// 		return;
	// 	}
	// 	for(int i = 0; i < depletion.size(); i++){
	// 		(this->machine_bins[i])->supply_level = (this->machine_bins[i])->supply_level-depletion[i];
	// 	}
	// 	return;
	// }
	void set_obs(double** obs){
		this->obs = obs;
	}
	int get_size_x(){
		return this->size_x;
	}
	int get_size_y(){
		return this->size_y;
	}
	vector<bin_node*> get_warehouse_bins(){
		return this->warehouse_bins;
	}
	vector<bin_node*> get_machine_bins(){
		return this->machine_bins;
	}
	double** get_obs(){
		return this->obs;
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
		vector<bin_node*> s = map->get_machine_bins();
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
	} else if((node->bin)->bin_type == "machine"){
		double curr_supply_level = ((node->bin)->supply_level);
		cout << "Machine Heuristic: ";
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
	cout << "    material: ";
	cout << bin->mat_type;
	cout << "    coords: (";
	cout << bin->x;
	cout << ", ";
	cout<< bin->y;
	cout << ")";
	cout << "    supply level: ";
	cout << bin->supply_level;
	cout << "\n";
}
void print_goal_node(goal_node* goal){
	print_bin_node(goal->bin);
	cout<< "Distance Travelled: ";
	cout<< goal->c;
	cout<< "    Heuristic: ";
	cout<< goal->h;
	cout << "\n";
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
		if((top_node->bin)->bin_type == "machine"){
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
			// Euclidean Distance
			// double dist = sqrt(pow(curr_x-last_x,2) + 
			// 		           pow(curr_y-last_y,2));
			// AStar Distance
			double dist = get_distance(last_x, last_y, curr_x, curr_y, map->get_size_x(), map->get_size_y(), map->get_obs());
			curr_node->c = top_node->c + dist;
			curr_node->h = compute_goal_heuristic(curr_node);
			curr_node->prev = top_node;
			open.push(curr_node);
		}
	}
	vector<goal_node*> ans;
	while(top_node->prev != NULL){
		print_goal_node(top_node);
		ans.insert(ans.begin(),top_node);
		top_node = top_node->prev;
	}
	ans.insert(ans.begin(), start);
	return ans; // returns vector of warehouse and machine
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
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
    } else if (nlhs != 3) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "Three output argument required."); 

	} /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    /* init map representation */
    Map* m = new Map();
    m->set_size(x_size, y_size);
    double** obs = (double**)malloc(sizeof(double)*x_size*y_size);
    cout << "OBSTACLE MAP: \n";
    for(int i = 0; i < x_size; i++){
    	double* curr_row = (double*)malloc(sizeof(double)*y_size);
    	for (int j = 0; j < y_size; j++){
    		curr_row[j] = map[i+ x_size * j];
    		cout << curr_row[j];
    		cout << " ";
    	}
    	obs[i] = curr_row;
    	cout << "\n";
    }
    m->set_obs(obs);

    // for numerical map implementation
    // for(int i = 0; i < x_size; i++){
    // 	for(int j = 0; j < y_size; j++){
    // 		double curr_val = map[j*y_size+i];
    // 		if(curr_val > 0){
    // 			m->add_machine_bin(to_string(abs(curr_val)), i, j);
    // 		} else if (curr_val < 0){
    // 			m->add_warehouse_bin(to_string(abs(curr_val)), i, j);
    // 		}
    // 	}
    // }

    Robot* r = new Robot();
    double* r_pos = mxGetPr(ROBOT_IN);
    r->update_state(r_pos[0], r_pos[1]);

    // Depletion implementation
    // y_size = mxGetN(DEPLETE);
    // double* deplete = mxGetPr(DEPLETE);
    // vector<double>depletion;
    // for(int i = 0; i < y_size; i++){
    // 	depletion.push_back(deplete[i]);
    // }
    // m->update_supply_level(depletion);
   
    x_size = mxGetM(MACHINE);
    y_size = mxGetN(MACHINE);
    double* machine = mxGetPr(MACHINE);

    cout << "\n";
    cout << "MACHINE IN: \n";
    for(int i = 0 ; i < y_size; i++){
    	int x = machine[i];
    	int y = machine[i+x_size];
    	cout << "Coords: ( ";
    	cout << x;
    	cout << ", ";
    	cout << y;
    	cout << ")";
    	string type = to_string(abs(machine[i+2*x_size]));
    	cout << "    MATTYPE: ";
    	cout << type;
    	double supply_level = machine[i+3*x_size];
    	cout << "    SUPPLY LEVEL: ";
    	cout << supply_level;
    	cout << "\n";
    	m->add_machine_bin(type, x, y, supply_level);
    }
    x_size = mxGetM(WAREHOUSE);
    y_size = mxGetN(WAREHOUSE);
    cout << x_size; cout << y_size;
    double* warehouse = mxGetPr(WAREHOUSE);
    cout << "\n";
    cout << "WAREHOUSE IN: \n";
    for(int i = 0; i < x_size; i++){
    	int x = warehouse[i];
    	int y = warehouse[i+x_size*1];
    	cout << "Coords: ( ";
    	cout << x;
    	cout << ", ";
    	cout << y;
    	cout << ")";
    	string type = to_string(abs(warehouse[i+x_size*2]));
    	cout << "    MATTYPE: ";
    	cout << type;
    	double supply_level = warehouse[i+x_size*3];
    	cout << "    SUPPLY LEVEL: ";
    	cout << supply_level;
    	cout << "\n";
    	m->add_warehouse_bin(type, x, y, supply_level);
    }

    vector<goal_node*> goal_plan = goal_planner(m,r);

    vector<double> x_plan;
    vector<double> y_plan;
    for(int i = 0; i < goal_plan.size()-1; i++){
    	int curr_x = (goal_plan[i]->bin)->x;
    	int curr_y = (goal_plan[i]->bin)->y;
    	int next_x = (goal_plan[i+1]->bin)->x;
    	int next_y = (goal_plan[i+1]->bin)->y;
    	vector<vector<int>> curr_plan = plan(curr_x, curr_y, next_x, next_y, m->get_size_x(), m->get_size_y(), m->get_obs());
    	cout << curr_plan.size();
    	for(int j = 0; j < curr_plan.size(); j++){
    		x_plan.push_back(curr_plan[j][0]);
    		y_plan.push_back(curr_plan[j][1]);
    	}
    }

    double* x_plan_db = (double*)malloc(sizeof(double)*x_plan.size());
    double* y_plan_db = (double*)malloc(sizeof(double)*y_plan.size());
    for(int i = 0 ; i < x_plan.size(); i++){
    	x_plan_db[i] = x_plan[i];
    	y_plan_db[i] = y_plan[i];
    }
    plhs[0] = mxCreateDoubleMatrix(1, x_plan.size(), mxREAL);
    memcpy(mxGetPr(plhs[0]), x_plan_db, sizeof(double)*x_plan.size());
    plhs[1] = mxCreateDoubleMatrix(1, y_plan.size(), mxREAL);
    memcpy(mxGetPr(plhs[1]), y_plan_db, sizeof(double)*y_plan.size());

    plhs[2] = mxCreateDoubleMatrix(1, goal_plan.size()*2, mxREAL);
    double* values = (double*)malloc(sizeof(double)*goal_plan.size()*2);
    for(int i = 0; i < goal_plan.size(); i++){
    	values[i*2] = (goal_plan[i]->bin)->x;
    	values[i*2+1] = (goal_plan[i]->bin)->y;
    }
    memcpy(mxGetPr(plhs[2]), values, sizeof(double)*2*goal_plan.size()*2);

    // plhs[0] = mxCreateDoubleMatrix(1, goal_plan.size()*2, mxREAL);
    // double* values = (double*)malloc(sizeof(double)*goal_plan.size()*2);
    // for(int i = 0; i < goal_plan.size(); i++){
    // 	values[i*2] = (goal_plan[i]->bin)->x;
    // 	values[i*2+1] = (goal_plan[i]->bin)->y;
    // }
    // memcpy(mxGetPr(plhs[0]), values, sizeof(double)*2*goal_plan.size()*2);
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
// 	m->add_machine_bin("A", 0, 0);
// 	m->add_machine_bin("A", 0, 7);
// 	m->add_machine_bin("B", 7, 0);
// 	m->add_machine_bin("B", 7, 7);
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