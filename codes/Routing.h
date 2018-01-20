#ifndef ROUTING_H_
#define ROUTING_H_

#include "head.h"
#include "event.h"
#include "solution.h"
#include "tabu.h"
#include "cluster.h"

struct GoodSolutions
{
	vector <Solution> solutions;
	int count;
	int limit;

	GoodSolutions()
	{
		count = 0;
		limit = GOOD_SOLUTION_LIMIT;
	}
};

class Routing
{
	friend class Vehicle;
	friend class Client;
	friend class Solution;
	friend Client get_node_from_event(int node_id);

private:
	enum {DYNAMIC_HANDLER, STATIC_HANDLER};

	Solution currentsolution;
	Solution tmpsolution;
	Cluster cluster[VEHICLE_LIMIT];
	Client pickups[CLIENT_LIMIT];
	Client deliverys[CLIENT_LIMIT];

	GoodSolutions good_solutions;

	int REACTION_TYPE;
	double EPSILON;

	bool EVENT_DATA_EXIST;

	// Note: these pointers all shares one instance, pointed by pevent
	Event *pevent[EVENT_LIMIT];
	Event *pevent_time[EVENT_LIMIT];
	Event *pevent_node[EVENT_LIMIT];
	queue <Event *> pevent_buffer;
	int event_count_time;
	int event_count_node;
	int event_time_ptr;

	// new id counter reserved for new arrival nodes
	int new_id;
	// current time when doing routing
	double current_time;
	double TIME_LIMIT;

	// time cost measurement
	double dynamic_time_cost;
	double static_time_cost;

	ofstream fdoutx;
	tabuhistory tahistory;

public:
	Routing()
	{
		REACTION_TYPE = DYNAMIC_HANDLER;
		int i;
		for (i = 0; i < EVENT_LIMIT; i++) pevent[i] = NULL;
		EPSILON = 1e-4;
	}
	~Routing()
	{
		int i;
		for (i = 0; i < EVENT_LIMIT; i++)
		{
			if (pevent[i] != NULL) delete pevent[i];
		}
	}
	void run(int caseid);

	void init();
	void event_handling_init(double time_static);
	void end_good_solutions();
	void file_input(int caseindex);
	void build_init_solution();
	void perform_tabu_search();
	void perform_opt();
	void perfrom_waste_distribution();
	void read_events(int caseid);
	void route_events_prev();
	void route_events_main();
	void end_case(int caseid);

	void get_reduced_demand();
	bool build_solution_direct();
	bool build_solution_clustering_init();
	bool build_solution_clustering();
	void write_generator_input(int caseid);
	bool build_cluster_init();
	// void shift_operator(Solution &tpsolution, Shiftrecord srd);
	void write(string info);

	// for routing events
	int assign_new_id();
	// get next upcoming event 
	// -- obsolete
	Event *get_next_event();
	// let vehicles move to next nodes, return new current time
	// -- obsolete
	double vehicles_move_to_next_nodes(double event_time);
	//
	Event *is_event_node(Client node);

	// check if a good solution becomes better than best solution
	void check_good_solutions();
	// compare two solutions
	int better_solution(int i, int j);
	// 
	void clear_event_pointers();
	//
	void check_min_time_gap();
	// update current time stamp
	void update_current_state(Event *current_event);

	//
	// void copy_event(Event *dst, Event *src);

	// event handlers -- obsolete
	// dynamic
	void NA_handler_dynamic(NodeArrival na_event);
	void NC_handler_dynamic(NodeCancellation nc_event);
	void NV_handler_dynamic(NegativeVariation nv_event);
	void PV_handler_dynamic(PositiveVariation pv_event);
	// static
	void NA_handler_static(NodeArrival na_event);
	void NC_handler_static(NodeCancellation nc_event);
	void NV_handler_static(NegativeVariation nv_event);
	void PV_handler_static(PositiveVariation pv_event);
};

#endif