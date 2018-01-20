#ifndef SOLUTION_H_
#define SOLUTION_H_

#include "head.h"
#include "tabu.h"
#include "event.h"

class AdditionalInfo
{
 public:
	int event_type;
	double event_time;
	int node_id;
	double time_taken;
	double cost;
	double avg_fillrate_delivery;
	double min_fillrate_delivery;
	double max_fillrate_delivery;
	double waste;

	AdditionalInfo(){}

	~AdditionalInfo(){}

	void set_value(double time_taken_, double cost_, double avg_fillrate_delivery_,
		double min_fillrate_delivery_, double max_fillrate_delivery_, double waste_,
		int event_type_, double event_time_, int node_id_)
	{
		time_taken = time_taken_;
		cost = cost_;
		avg_fillrate_delivery = avg_fillrate_delivery_;
		min_fillrate_delivery = min_fillrate_delivery_;
		max_fillrate_delivery = max_fillrate_delivery_;
		waste = waste_;
		event_type = event_type_;
		event_time = event_time_;
		node_id = node_id_;
	}

	void write(ofstream &fdoutx);
};

// ---- Solution class ----
class Solution
{
	friend class Vehicle;
public:
	enum {DYNAMIC_HANDLER, STATIC_HANDLER};

	Vehicle vehiclelist[VEHICLE_LIMIT];

	// How many clients left unserved?
	int unservedclient;
	int unservedlist[CLIENT_LIMIT];

	// cost of all the vehicle of the solution
	double solutioncost;

	// evaluation function
	double evalfunc;

	// modified evaluation function
	double mevalfunc;

	// is this solution feasible?
	bool isfea;

	// Shift history
	/*
	shiftrecord *shifthistory;
	int historylen;
	*/

	// Sequential table
	// Suppose we have 500 nodes, 20 vehicles, nodes goes equaly to every vehicle, then it will be 25 in each node.
	// In this case, the total possibility will be 25 * 475 * 20 = 237500. We make the table 300000, which will be sufficient
	Shiftrecord sttable[300000];
	int stp;
	int possibilities;

	// Is the solution only one vehicle?
	bool onevehicle;

	// ---- data for dynamic routing, stage v ----
	// is this solution have deleted nodes?
	// const double *pcurrent_time;



	Client dvrp_node_set[CLIENT_LIMIT];
	int dvrp_node_count;
	bool unserved_dvrp;

	int REACTION_TYPE;
	double TIME_LIMIT;
	double EPSILON;

	// end time of this solution
	double end_time;

	double current_time;
	// int deleted_nodes_count;
	// Client deleted_nodes[CLIENT_LIMIT];
	double waste;

	double equity;

	double dynamic_time_cost;
	double static_time_cost;

	//
	bool isfea_after_current;

	// all solutions have their private event list
	Event *pevent_time[EVENT_LIMIT];
	Event *pevent_node[EVENT_LIMIT];
	priority_queue <Event *, vector <Event *>, EventComparePtr> pevent_buffer;
	int event_count_time;
	int event_count_node;
	int event_time_ptr;

	AdditionalInfo info_list[EVENT_LIMIT];
	int info_count;

	ofstream *pfdoutx;

	// constructor
	Solution();

	// destructor
	~Solution(){}

	// initialization
	void init(Vehicle *vehiclelist); 

	// evaluation
	// this function will calculate solutioncost, evalfunc and mevalfunc
	void eval();

	// feasibility check reload
	// return 0: feasible, 1: not feasible
	int feasibility_check();

	// Check the history
	bool check_operation_record(Shiftrecord srd);

	// Insert the operation in the history
	void insert_operation_record(Shiftrecord srd);

	// choose shift operation
	// Note that member function choose_shift will do four things
		// 1> randomly choose a shift -- to be implemented
			// Note that we have two restrictions for the choice
			// First, the chioce should not be in the tabu history. 
			// To ensure this, we have a tabu history table, which is implemented by the class tabu history
			// Second, we should not choose the operation we have performed before. 
			// To ensure this, we have a operation history list in che class solution
		// 2> Update the current solution operation history
		// 3> If this operation is tabu, we will update tabu history. We will leave this for the later tabu check procedure.
		// 4> If we have checked all the shift option, can not find a better or tabu, then we will terminate the iteration.
	Shiftrecord choose_shift(tabuhistory history);

	// Choose shift operation for sequential tabu
	Shiftrecord choose_shift_st(tabuhistory history); // st stands for sequential tabu // --ok

	// Set sequential shift table
	void set_sttable(); // --ok

	// update: clean operation history
	void update();

	// Show history
	void show_history();

	// print the solution
	void show() const;
	void write(ofstream &fdoutx);

	// perform rearrange operator
	void perform_rearrange();

	// perform swap operator
	void perform_swap();

	// perform opt operator
	void perform_opt();

	// Waste redistribute
	bool distribute_waste();

	// join two solutions
	void join(Solution &tmpsolution);

	// ---- Functions for routing events ----

	//
	void write_generator_input(ofstream &fd_generator);

	//
	void init();

	//
	void event_handling_init();

	//
	void clear_event_pointers();

	// when a node arrival, insert it into the solution
	// -- if find feasible insertion, insert it
	// -- else, construct a new route -- may fail
	// return true on best feasible insertion or successfully new route construction
	// return false on no available vechicle for route construction
	bool insert_event_na(NodeArrival na_event);

	// if no feasible insersion, find best infeasible insertion
	void find_best_infeasible_insert(Client node);

	// relax demand on node cancellation or negative variation
	// only influence nodes after current time
	void relax_demand_and_reload();

	// perform operator chain
	// -- return value: if feasible
	bool operator_chain();

	// the following three operators are basically the same as before
	// except we need to consider current time
	void perform_rearrange_dvrp();
	void perform_swap_dvrp();
	void perform_opt_dvrp();

	// construct a new route on node arrival
	// -- if no available vehicle left, abandon this event
	bool construct_on_arrival(Client node, double start_time);

	// given event time
	// let the vehicles move to next nodes
	// induce new current time
	double vehicles_move_to_next_current(double event_time);

	// given new current time, let vehicles move to next nodes
	void vehicles_follow_next_current(double new_current_time);

	// let best solution push forward time
	double push_forward_time();

	//
	bool vehicles_finished_routing(const Event *p);

	// let good solutions follow next current time
	// if next_current_time == -1, push to end
	void follow_next_current(double next_current_time);

	// do batch event handling
	void batch_event_handling();

	//
	Event *get_next_upcoming_event();

	//
	Event *get_next_event();

	// get nearest node based event
	Event *get_next_event_node();

	// get nearest time based event
	Event *get_next_event_time();

	//
	Event *is_event_node(Client node);

	//
	void change_supply(Event *p);

	// collect nodes that have not been visited yet into dvrp_node_set
	// for future route construction using static method
	void collect_nodes();

	//
	void add_into_node_set(Client node);

	//
	void insert_node_sdvrp(Client node);

	//
	void rebuild();

	//
	void route_from_current();

	//
	void insert_unserved_nodes();

	//
	void set_sttable_after_current();

	//
	int feasibility_check_sdvrp();

	//
	void tabu_search_after_current();

	//
	void opt_after_current();

	// 
	void calculate_equity();

	//
	void waste_distribute_after_current();

	//
	void end_static_handler();

	// solution level event handlers
	// dynamic
	void NA_handler_dynamic(NodeArrival &na_event);
	void NC_handler_dynamic(const NodeCancellation &nc_event);
	void NV_handler_dynamic(const NegativeVariation &nv_event);
	void PV_handler_dynamic(const PositiveVariation &pv_event);
	// static
	void NA_handler_static(NodeArrival na_event);
	void NC_handler_static(NodeCancellation nc_event);
	void NV_handler_static(NegativeVariation nv_event);
	void PV_handler_static(PositiveVariation pv_event);

	// 
	void find_unhandled_events();

	// 
	int find_event_vehicle(int node_id);

	void get_fillrate(double &avg_fillrate_delivery, 
		double &min_fillrate_delivery, double &max_fillrate_delivery);

	void get_waste(double &tmpwaste);

	void write_info(ofstream & fdoutx);
};

void copy_sttable(Solution &sour, Solution &dest);

#endif