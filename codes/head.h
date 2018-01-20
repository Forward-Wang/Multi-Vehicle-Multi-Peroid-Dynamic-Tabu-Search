#ifndef HEAD_H_
#define HEAD_H_

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <io.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <ctime>
#include <limits>
#include <random>
#include <functional>
#include <queue>

#include "event.h"
#include "cluster.h"

using namespace std;

// ---- General definition ----
#define VEHICLE_LIMIT 15
#define CLIENT_LIMIT 250
#define SOLUTION_LIMIT 10
#define SHIFT_LIMIT 1000000
#define EVENT_LIMIT 200

// route construction
#define RELAX_RATIO 0.8

// tabu search 
#define TABUTENURE 8
#define STOPCRITERIA 2000

// this type define will consist with previous files
#define PICK 1
#define DELV 0

// event handling
#define TIME_RELAXATION 1.25
#define GOOD_SOLUTION_LIMIT 5


// ---- client class ----
// Note: we will view depot as pick-up type
class Client
{
public:
	static int totalnode;

	// Basic variable -- read them from the file
	int id;
	int type;
	double xcoordinate;
	double ycoordinate;
	double demand;
	double long_supply;
	double prsb_supply;
	int frequency; 
	double opentime;
	double closetime;
	double servicetime;

	// auxiliary variables
	// load of node
	// long-time and perishable
	double load;
	double long_load;
	double prsb_load;
	// delivered demand
	double long_delivered;
	double prsb_delivered;
	double fillrate;
	// is this a new arrival node?
	bool newarrival;

	// This is used both in route construction and clustering
	bool isinserted;

	// cost = the time vehicle reaches the node + service time at that node
	double cost;

	// structural variable -- next position in the linked list
	int next;

	// distance from the center, for k-clustering algorithm
	double distance;
   
	// for stage iii --  fair allocation
	double init_demand;
	double reduced_demand;


	Client();
	// Client(NewArrivalEvent naevt);
	~Client(){}

	int loadcargo(Client &clientfrom, double capacity);
	int deliver(Client &clientto);
  void count_distance();
  void show() const;
	void write(ofstream &fdoutx) const;

	inline void calculate_fillrate();

	// ---- dynamic VRP ----
	void init(NodeArrival na_event);
	int deliver_saturated(Client &clientto);
	int loadcargo_saturated(Client &clientfrom, double capacity);
	void change_supply(Event *p);
};

// ---- client level auxiliary function ----
int clientcmp(const void *a, const void *b);
double get_dist(Client &cltx, Client &clty);

class Cluster;

// ---- Vehicle class ----
class Vehicle
{
public:
	static int totalvehicle;

	// Basic variable of vehicle
	int id;
	double capacity;
	double opentime;
	double closetime;
	double relaxted_closetime;
	
	// route of this vehicle
	Client route[CLIENT_LIMIT]; // Attention here
	int head;
	int tail;
	int len;
    
	// veriable for all stages
	// Final load
	double finalload; // -- obsolete
	double long_finalload;
	double prsb_finalload;
	// this shows the equity 
	double equity;

	// Total time cost
	double totalcost;

	// veriable for stage ii
	// capacity violate
	double p; 
	// load violate
	double q;
	// tour duration violate
	double t;
	// is feasible?
	bool isfea;

  // len is all total node the route goes through
	// note: if len == 2, means that this vehicle has not a route, this could happen

	/*
    // two mode of the routelist: linked list and array
    // linked list applies fine for step3 and step5 (easy to insert)
    // while array applies fine for step4 and step6 (easy to swap and locate)
	*/

	// variables for dynamic routing
	// need to maintain these two variables
	double *pcurrent_time;
	// is this route feasible after current time?
	// include the next visiting node
	bool isfea_after_current;
	// is this route feasible after next visiting route?
	// exclude the next visiting node
	// bool isfea_after_next_visiting;

	Vehicle()
	{
		isfea_after_current = true;
	}
	~Vehicle(){}

	void init(Client *client);
	void init(Cluster &tmpcluster);

	int feasibility_check(int node, int position); // only for step3
	int feasibility_check(); // function reload for step4, 5, 6
	// here are the return value of feasibility_check function:
	// 0: feasible; 1: violate feasibility criteria 1,2,3; -1: violate feasibility criteria4

	double cost_calculate(int node, int position); // for step 3
    
	// update the cost and load, for step 4, 5, 6
	void update();
    
	// print the route
	void show() const;
	void show_jn() const;
	void write(ofstream &fdoutx) const;

	// insert function for step 3
	void insert(int node, int position);

	// insert function for step 5
	void insert_i(int id, int position);
	void insert_s(Client nd, int position);
	// NodeInsertRecord insert_na(Client clt, int position);

	// kickout function for step 5
	void kickout(int position);
	// remove one node
	// a combination of kickout, linkedlist2array and update
	void remove(int position);

	// for step4, 5, 6
	void linkedlist2array();

	// for step4, rearrange operator
	void swap(int i, int j);
	bool rearrange_operator();
	void swap_operator();

	// For stage iii
	bool opt_operator(int i, int j);

	// Waste re-distribution
	bool distribute_waste();

	// joint with another route
	bool join(Vehicle &tmpvehicle);

	// ---- functions for stage v ----

	// next visiting location
	int next_visiting_loc(double current_time);

	// initialize on node arrival
	void init_na(double start_time, int id);

	// insert a new arrival node
	void insert_na(Client nd, int position);

	// basically the same as insert_s, but use update_sdvrp
	void insert_sdvrp(Client nd, int position);

	// update function for dynamic VRP
	void update_dvrp();

	// do not use saturated load& deliver
	// keep isfea_after_next_visiting
	void update_sdvrp();

	//
	void calculate_equity();

	//
	void rearrange_operator_dvrp(double current_time);

	// 
	bool opt_operator_dvrp(int i, int j);

	//
	void waste_distribute_after_current();

	//
	int feasibility_check_sdvrp();
};

// ---- vehicle level auxiliary function ----
bool swap_operator(Vehicle &vehiclei, Vehicle &vehiclej, int i, int j);

// ---- cluster ----
class Cluster
{
public:
	Client clstclient[CLIENT_LIMIT];
	Client picks[CLIENT_LIMIT];
	Client delvs[CLIENT_LIMIT];
	Client head;
	Client tail;

	int len;
	double time_reduce;

	double centerx;
	double centery;

	double totalpickdemand;
	double totaldelvdemand;

	// Constructor
	Cluster();

	// Destructor
	~Cluster(){}

	void set_head_tail();
	void push(Client clt);
	void pop(int pos);
	void show() const;
	void write();
	void erase();
	void find_center();
	void operator += (const Cluster &clst);
};




#endif