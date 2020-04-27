#include"symbolic_functions.cpp"

void add_others(Env* env, int num_materials, int num_supplies){
    for (int i = 0; i < num_materials; i++){
        string curr_sym = "M" + to_string(i);
        env->add_symbols(curr_sym);
        GroundedCondition gc = GroundedCondition("Material", curr_sym, true);
        env->add_initial_condition(gc);
        GroundedCondition gc = GroundedCondition("NotRobot", curr_sym, true);
        env->add_initial_condition(gc);
    }
    for(int i = 0; i < num_supplies; i++){
        string curr_sym = "S" + to_string(i);
        env->add_symbols(curr_sym);
        GroundedCondition gc = GroundedCondition("Supply", curr_sym, true);
        env->add_initial_condition(gc);
        GroundedCondition gc = GroundedCondition("NotRobot", curr_sym, true);
        env->add_initial_condition(gc);
    }
}

double heuristic(Env* env, node* curr, bool use){
    double h = 0;
    if use{
        if(!(curr->last_action).get_name() == "Move" || !(curr->last_action).get_name() == "MoveTo"){
            h = 300; //assume all other actions take 5 mins
        } else {
            list<string> arguments = (curr->last_action)->get_arg_values();
            list<string>::iterator it = arguments.begin();
            advance(it, 1);
            tuple<int,int> start = env->m[*it];
            advance(it,1);
            tuple<int,int> goal = env->m[*it];
            h = sqrt(pow((get<0>(start)-get<0>(goal)),2) + pow((get<1>(start)-get<1>(goal))));
        }
    }
    return h;
}


int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    add_others(env);
    int num_supply_bins = 2;
    int num_warehouse_spots = 2;
    int num_robots = 1;
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}