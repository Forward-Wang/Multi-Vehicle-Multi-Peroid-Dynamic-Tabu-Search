#include "head.h"
#include "solution.h"

#define _CRT_SECURE_NO_WARNINGS

// tabu parameter 
extern double lumbda;
extern double alpha;
extern double beta;
extern double gamma;

extern Solution bestsolution;
extern tabuhistory tahistory;
// extern Routing routing;

void shift_operator(Solution &tpsolution, Shiftrecord srd);
void shift_operator_sdvrp(Solution &tpsolution, Shiftrecord srd);

// ---- constructor ----
Solution::Solution()
{
	// How many clients left unserved?
	unservedclient = 0;

	// cost of all the vehicle of the solution
	solutioncost = 0;

	// evaluation function
	evalfunc = 0;

	// modified evaluation function
	mevalfunc = 0;

	// Shift history
	stp = 0;
	possibilities = 0;

	// Is the solution only one vehicle?
	onevehicle = false;

	info_count = 0;
}

// ---- rearrange operator ----
void Solution::perform_rearrange()
{
	printf("start to perform rearrange...\n");
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		printf("vehicle %d, length %d\n", i, vehiclelist[i].len);
		if (vehiclelist[i].len > 2)
		{
			vehiclelist[i].rearrange_operator();
		}
	}
	printf("Rearrange operator finished, store as bestsolution!\n");
	show();
}

// ---- swap operator ----
void Solution::perform_swap()
{
	int i, j;
	for (i = 1; i <= Vehicle::totalvehicle - 1; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		for (j = i + 1; j <= Vehicle::totalvehicle; j++)
		{
			if (vehiclelist[j].len <= 2) continue;
			swap_operator(vehiclelist[i], vehiclelist[j], i, j);
		}
	}
	printf("Swap operator finished, store as bestsolution!\n");
	show();
}

// ---- opt operator ----
void Solution::perform_opt()
{}

// ---- evaluation ----
// nothing to do with current time, leaving all issues relatet to current time to the vehicle level functions
void Solution::eval()
{
	int i;
	double retaintime = 0;
	solutioncost = 0;
	evalfunc = 0;
	mevalfunc = 0;
	isfea_after_current = true;
	end_time = -1;
	waste = 0;

	// Calculate evaluate function and solution cost
	// We assume p, q, t had already been calculated in vehilceroute member function update(), which is what we actually do
	// We also assume every time we call eval(), we have already called update() before, which is also what we actually do
	for (i = 1; i <= VEHICLE_LIMIT; i++)
	{
		if (vehiclelist[i].len <= 2) // if this vehicle is not in use, skip
		{
			// printf("jumping vehicle %d\n", i);
			continue;
		}
		solutioncost += vehiclelist[i].totalcost;
		// printf("solution evaluation, route cost = %lf\n", vehiclelist[i].vehicleroute.totalcost);
		// note that p, q, t is calculated in vehiclie route member function update() 
		// Please check the definition here is right
		evalfunc += vehiclelist[i].totalcost + alpha * vehiclelist[i].p +
			beta * vehiclelist[i].q + gamma * vehiclelist[i].t;
		if (vehiclelist[i].totalcost > end_time) end_time = vehiclelist[i].totalcost;
		if (vehiclelist[i].isfea_after_current == false)
		{
			// debug:
			// printf("vehicle %d is not feasible after currrent!\n", i);
 			isfea_after_current = false;
		}
		waste += vehiclelist[i].finalload;
	}

	// Calculate retain time for modified evaluate function
	for (i = 0; i < tahistory.total; i++)
	{
		retaintime += tahistory.history[i].retaintime;
	}

	calculate_equity();

	// Note here we use the solutioncost of best solution to calculate modified evaluate function
	// Please check out the definition here is right
	mevalfunc = evalfunc + lumbda * bestsolution.solutioncost * sqrt(Vehicle::totalvehicle * Client::totalnode) * retaintime;
	if (solutioncost == evalfunc) isfea = true;
	else isfea = false;
}

// ---- print the solution ----
void Solution::show() const
{
	int i;
	double time_consumption;
	if (REACTION_TYPE == STATIC_HANDLER) time_consumption = static_time_cost;
	else time_consumption = dynamic_time_cost;
	printf("total vehicle %d\n", Vehicle::totalvehicle);
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		printf("vehicle %d, len = %d\n", i, vehiclelist[i].len);
		if (vehiclelist[i].len > 2)
		{
			printf("Vehicle %d\n", i);
			vehiclelist[i].show();
			printf("p:%lf, q:%lf, t:%lf\n", vehiclelist[i].p, vehiclelist[i].q, vehiclelist[i].t);
		}
	}
	printf("Total cost of the solution:%lf, eval: %lf, equity: %lf\n", solutioncost, evalfunc, equity);
	printf("Time consume of handlers: %lf\n", time_consumption);
}

// ---- write the solution ----
void Solution::write(ofstream &fdoutx)
{
	int i;
	char outbuffer[500];
	double time_consumption;
	if (REACTION_TYPE == STATIC_HANDLER) time_consumption = static_time_cost;
	else time_consumption = dynamic_time_cost;
	sprintf(outbuffer, "total vehicle %d\n", Vehicle::totalvehicle);
	fdoutx << outbuffer;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		sprintf(outbuffer, "vehicle %d, len = %d\n", i, vehiclelist[i].len);
		fdoutx << outbuffer;
		if (vehiclelist[i].len > 2)
		{
			sprintf(outbuffer, "Vehicle %d\n", i);
			fdoutx << outbuffer;
			vehiclelist[i].write(fdoutx);
			sprintf(outbuffer, "p:%lf, q:%lf, t:%lf\n", vehiclelist[i].p, vehiclelist[i].q, vehiclelist[i].t);
			fdoutx << outbuffer;
		}
	}
	sprintf(outbuffer, "Total cost of the solution:%lf, eval: %lf, equity: = %lf\n", solutioncost, evalfunc, equity);
	fdoutx << outbuffer;
	sprintf(outbuffer, "Time consume of handlers: %lf\n", time_consumption);
	fdoutx << outbuffer;
}

// ---- set sequential tabu table ----
void Solution::set_sttable()
{
	int i, j, k, l, m;
	onevehicle = true;
	stp = 0;
	m = 0;
	printf("Setting sttable...\n");
	// printf("length of vehicle 10 = %d\n", vehiclelist[10].len);
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		// printf("length of vehicle %d = %d\n", i, vehiclelist[i].len);
		if (vehiclelist[i].len <= 2) continue;
		for (j = 1; j <= Vehicle::totalvehicle; j++)
		{
			if (vehiclelist[j].len <= 2 || j == i) continue;
			for (k = 1; k <= vehiclelist[i].len - 2; k++)
			{
				onevehicle = false;
				for (l = 0; l <= vehiclelist[j].len - 2; l++)
				{
					sttable[m].vehiclefromid = i;
					sttable[m].vehicletoid = j;
					sttable[m].nodepos = k;
					sttable[m].nodeid = vehiclelist[i].route[k].id;
					sttable[m].position = l;
					sttable[m].isvalid = true;
					m++;
					if (vehiclelist[10].len != 2)
					{
						system("pause");
					}
				}
			}
		}
	}
	printf("Set sttable, length of sttable: %d\n", m);
	possibilities = m;
	return;
}

// ---- choose a shift ---- 
Shiftrecord Solution::choose_shift_st(tabuhistory history)
{
	Shiftrecord shiftobj;
	if (onevehicle == true)
	{
		shiftobj.isvalid = false;
		return shiftobj;
	}
	stp++;

	while (history.check_record(sttable[stp - 1]) == true)
	{
		stp++;
	}
	if (stp == possibilities)
	{
		printf("All possible shift try out. total");
		printf("stp: %d\n", stp);
		shiftobj.isvalid = false;
		return shiftobj;
	}
	shiftobj = sttable[stp - 1];
	return sttable[stp - 1];
}

// ---- check feasibility ----
int Solution::feasibility_check()
{
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		if (vehiclelist[i].feasibility_check() != 0) return 1;
	}
	// Bug detecting code
	if (solutioncost != evalfunc)
	{
		printf("A illegal route!\n");
		show();
		while (1);
		exit(1);
	}
	return 0;
}

// ---- merge this solution with another ----
void Solution::join(Solution &tmpsolution)
{
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2 && tmpsolution.vehiclelist[i].len <= 2) continue;
		vehiclelist[i].join(tmpsolution.vehiclelist[i]);
	}
	eval();
	return;
}



// ---- functions for dynamic routing, stage v ----

void Solution::write_generator_input(ofstream &fd_generator)
{
	int i;
	int j;
	char buff[512];
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		for (j = 1; j < vehiclelist[i].len - 1; j++)
		{
			if (vehiclelist[i].route[j].type == DELV) continue;
			sprintf(buff, "%d %f %f %f\n",
				vehiclelist[i].route[j].id,
				vehiclelist[i].route[j].long_supply,
				vehiclelist[i].route[j].prsb_supply,
				vehiclelist[i].route[j].cost );
			fd_generator << buff;
		}
	}
	return;
}

void Solution::init()
{
	// How many clients left unserved?
	unservedclient = 0;

	// cost of all the vehicle of the solution
	solutioncost = 0;

	// evaluation function
	evalfunc = 0;

	// modified evaluation function
	mevalfunc = 0;

	// Shift history
	stp = 0;
	possibilities = 0;

	// Is the solution only one vehicle?
	onevehicle = false;

	info_count = 0;

	//
	clear_event_pointers();
}

void Solution::event_handling_init()
{
	int i;
	static_time_cost = 0;
	dynamic_time_cost = 0;

	// loose time limits 
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		vehiclelist[i].closetime *= TIME_RELAXATION;
		vehiclelist[i].pcurrent_time = &current_time;
		if (vehiclelist[i].len > 2) vehiclelist[i].update_dvrp();
	}

	// set pointers
	for (i = 0; i < EVENT_LIMIT; i++)
	{
		pevent_node[i] = NULL;
		pevent_time[i] = NULL;
	}

	event_count_node = 0;
	event_count_time = 0;
	event_time_ptr = 0;
	waste = 0;
	eval();
	return;
}

void Solution::clear_event_pointers()
{
	// clear event pointer
	int i;
	for (i = 0; i < EVENT_LIMIT; i++)
	{
		if (pevent_node[i] != NULL)
		{
			delete pevent_node[i];
		}
		if (pevent_time[i] != NULL)
		{
			delete pevent_time[i];
		}
		pevent_node[i] = NULL;
		pevent_time[i] = NULL;
	}
}

// insert the now arrival node on node arrival event
// -- try to find the best feasible insertion, if best feasible, return ture
// -- if no best feasible insertion, construct a new route, then perform operator chain, return true
// -- else, cannot construct a new route, return false, handler failure
// note that, since we are assuming infinity number of vehicles, this function never returns false
bool Solution::insert_event_na(NodeArrival na_event)
{
	int i, j;
	int next_loc;
	NodeInsertRecord tmp_insert;
	NodeInsertRecord best_insert;
	Vehicle *tmpvehicle = new Vehicle();
	// Vehicle tmpvehicle;
	Client tmpclient;
	bool tmpret;
	bool vehicles_all_finished;

	// initialize the new arrival client
	tmpclient.id = na_event.node_id;
	tmpclient.type = PICK;
	tmpclient.xcoordinate = na_event.x;
	tmpclient.ycoordinate = na_event.y;
	tmpclient.prsb_supply = na_event.psupply;
	tmpclient.long_supply = na_event.lsupply;

	tmp_insert.valid = false;
	best_insert.valid = false;
	best_insert.cost_increase = TIME_LIMIT;

	vehicles_all_finished = true;

	// for every route in this solution
	for (i = 1; i < Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		// find out the next node to be visit 
		next_loc = vehiclelist[i].next_visiting_loc(current_time);

		// this vehicle has already finished routing
		// try next vehicle
		/*if (i == 4)
		{
			printf("current time = %lf\n", current_time);
			printf("event time: %lf\n", na_event.time);
			printf("vehicle finish time: %lf\n", vehiclelist[i].totalcost);
			system("pause");
		}*/
		if (next_loc == -1 || next_loc == vehiclelist[i].len - 1)
		{
			printf("vehicle %d is finished, cannot insert!\n", i);
			continue;
		}
		else
		{
			vehicles_all_finished = false;
		}

		// still possible insertion locations
		// (*tmpvehicle) = vehiclelist[i];
		
		// from this node, try all insertions, find the best feasible
		for (j = next_loc; j < vehiclelist[i].len - 1; j++)
		{
			*tmpvehicle = vehiclelist[i];
			tmp_insert.vehicleid = i;
			tmp_insert.position = j;

			(*tmpvehicle).insert_na(tmpclient, tmp_insert.position);
			// need to modify: if this vehicle is feasible after this current time
			tmp_insert.valid = (*tmpvehicle).isfea_after_current;
			tmp_insert.cost_increase = (*tmpvehicle).totalcost - vehiclelist[i].totalcost;

			// the valid bit is correcponding to feasibility
			if (tmp_insert.valid == true && best_insert.valid == false)
			{
				best_insert = tmp_insert;
			}
			else if (tmp_insert.valid == true && best_insert.valid == true)
			{
				// a better insert with lower cost increase
				if (tmp_insert.cost_increase < best_insert.cost_increase)
				{
					best_insert = tmp_insert;
				}
			}
			else if(tmp_insert.valid == false && best_insert.valid == false) // tmp_insert is not valid
			{
				if (tmp_insert.cost_increase < best_insert.cost_increase)
				{
					best_insert = tmp_insert;
				}
			}
			else // tmp insert not valid, best insert is valid
			{
				;
			}
		}
	}

	// after previous procedure, we may have a best insert or not
	// if not feasible, construct a new route
	
	// for test, make it not feasible on purpose
	// best_insert.valid = false
	if (best_insert.valid == false)
	{
		// must have a best insert
		if (best_insert.cost_increase == TIME_LIMIT)
		{
			printf("did not find best insert! vehicles all finished = %d\n", vehicles_all_finished);
			delete tmpvehicle;
			return false;
			// do nothing
			goto END_INSERT;
			system("pause");
		}
		// -- TESTED
		printf("best insert is not valid, use operator chain\n");
		printf("insert this new node in vehicle %d, position %d\n", best_insert.vehicleid, best_insert.position);
		vehiclelist[best_insert.vehicleid].insert_na(tmpclient, best_insert.position);
		operator_chain();
		//tmpret = construct_on_arrival(tmpclient, na_event.time);
		//if (tmpret == false)
		//{
		//	printf("cannot construct new route on this node arrival: no available vehicle left!\n");
		//	printf("please consider raising the number of vehicles!\n");
		//	printf("saving the current stage .... \n");
		//	// save current stage:
		//	// TBC
		//	system("pause");
		//	return false;
		//}
		//// success, perform operator chain
		//else
		//{
		//	operator_chain();
		//}
	}
	// we have best insertion
	else
	{
		printf("best insert is valid!\n");
		printf("insert this new node in vehicle %d, position %d\n", best_insert.vehicleid, best_insert.position);
		// vehiclelist[best_insert.vehicleid].show();
		vehiclelist[best_insert.vehicleid].insert_na(tmpclient, best_insert.position);
		// vehiclelist[best_insert.vehicleid].show();
		eval();
	}
END_INSERT:
	delete tmpvehicle;
	return true;
}

// if we cannot find a best feasible insert, we will find an infeasible insert with lowest cost increase
void Solution::find_best_infeasible_insert(Client node)
{

}

// construct a node on arrival
bool Solution::construct_on_arrival(Client node, double start_time)
{
	int i;
	bool tmpret = false;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		// a unused vehicle
		if (vehiclelist[i].len <= 2)
		{
			vehiclelist[i].init_na(start_time, i);
			vehiclelist[i].insert_na(node, 0);
			tmpret = true;
			break;
		}
	}

	// BUG DETECTION: are these vehicles initialized?
	if (tmpret == false)
	{
		printf("no empty vehicle left! please increase the number of vehicle\n");
		system("pause");
	}
	return tmpret;
}

// ********************************************************************************************
// specification: using operators:
// operators:
// -- rearrange: 
// -- swap:
// -- 2-opt:
// start: a solution in any state
// -- first use shift operator, then swap operator, then 2-opt operator
// -- during the above procedure, the state of a vehicle may go through: unfeasible -- feasible -- feasible and better
// loop: perform operators:  
// -- if the current state of this solution is infeasible:
//   -- if this operator makes this solution feasible, update the state of the solution to be feasible
//   -- if not feasible, continue operators
// -- if the current state of this solution is feasible:
//   -- if this operator makes this solution better: update the state of this solution
//   -- else, continue operators
// end loop
// end state: |                dvrp-feasible               |        not dvrp-feasible        |
//            | better dvrp-feasible | worse dvrp-feasible |    no change in this solution   |
// dvrp-feasible = feasible after current time
// ********************************************************************************************

// -- TESTED
bool Solution::operator_chain()
{
	perform_rearrange_dvrp();
	perform_swap_dvrp();
	perform_opt_dvrp();
	if (isfea_after_current)
	{
		printf("after operator chain, the solution is feasible again!\n");
		return isfea_after_current;
	}
	else
	{
		printf("operator chain feels so sorry .... \n");
		return isfea_after_current;
	}
}

// this operator is intra-vehicle level
// -- GOOD 
// -- TESTED
void Solution::perform_rearrange_dvrp()
{
	printf("start to perform rearrange...\n");
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		printf("vehicle %d, length %d\n", i, vehiclelist[i].len);
		if (vehiclelist[i].len > 2)
		{
			vehiclelist[i].rearrange_operator_dvrp(current_time);
		}
	}
	eval();
	printf("Rearrange operator finished!\n");
	// show();
}

static void swap_operator_dvrp(Vehicle &vehiclei, Vehicle &vehiclej, double current_time);

// inter-vehicle level 
void Solution::perform_swap_dvrp()
{
	int i, j;
	for (i = 1; i <= Vehicle::totalvehicle - 1; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		for (j = i + 1; j <= Vehicle::totalvehicle; j++)
		{
			if (vehiclelist[j].len <= 2) continue;
			swap_operator_dvrp(vehiclelist[i], vehiclelist[j], current_time);
		}
	}
	eval();
	printf("Swap operator finished!\n");
	// show();
}

// swap two nodes in two different vehicles
// -- GOOD
static void swap_operator_dvrp(Vehicle &vehiclei, Vehicle &vehiclej, double current_time)
{
	// initialization
	// printf("swapping vehicle: i = %d, j = %d\n", i, j);
	Vehicle *tmpvehiclei = new Vehicle();
	Vehicle *tmpvehiclej = new Vehicle();
	Client tmpclientk;
	Client tmpclientl;
	int tmpnodek = -1;
	int tmpnodel = -1;
	int kstart = CLIENT_LIMIT;
	int lstart = CLIENT_LIMIT;
	double tmpcost = vehiclei.totalcost + vehiclej.totalcost;
	int k, l, i;

	// find out kstart and lstart
	for (i = 1; i < vehiclei.len; i++)
	{
		if (vehiclei.route[i].cost > current_time)
		{
			kstart = i + 1;
			break;
		}
	}
	for (i = 1; i < vehiclej.len; i++)
	{
		if (vehiclej.route[i].cost > current_time)
		{
			lstart = i + 1;
			break;
		}
	}

	// for every remained node k in route of vehicle i;
	for (k = kstart; k < vehiclei.len - 1; k++)
	{
		// and each remained node l in route of vehicle j
		for (l = lstart; l < vehiclej.len - 1; l++)
		{
			// printf("swapping: vehicle %d, position %d - vehicle %d, position %d\n", i , k, j, l);
			(*tmpvehiclei) = vehiclei;
			(*tmpvehiclej) = vehiclej;
			// store the intermediate node
			tmpclientk = (*tmpvehiclei).route[k];
			tmpclientl = (*tmpvehiclej).route[l];

			// make swap in position k, vehicle i
			(*tmpvehiclei).route[k] = tmpclientl;
			(*tmpvehiclei).route[k].next = k + 1;
			(*tmpvehiclei).update_dvrp();

			// make swap in position l, vehicle j
			(*tmpvehiclej).route[l] = tmpclientk;
			(*tmpvehiclej).route[l].next = l + 1;
			(*tmpvehiclej).update_dvrp();

			if ( (*tmpvehiclei).isfea_after_current == true && (*tmpvehiclej).isfea_after_current == true)
			{
				// printf("cost prev: %lf | now: %lf \n", tmpcost, tmpvehiclei.totalcost + tmpvehiclej.totalcost);
				if (( (*tmpvehiclei).totalcost + (*tmpvehiclej).totalcost) < tmpcost)
				{
					printf("find a swap: vehicle %d, position %d - vehicle %d, position %d\n", vehiclei.id, k, vehiclej.id, l);
					//printf("before\n");
					//printf("vehicle %d\n", i);
					//vehiclei.vehicleroute.show();
					//printf("vehicle %d\n", j);
					//vehiclej.vehicleroute.show();

					//printf("after\n");
					//printf("vehicle %d\n", i);
					//tmpvehiclei.vehicleroute.show();
					//printf("vehicle %d\n", j);
					//tmpvehiclej.vehicleroute.show();

					printf("cost reduced: %f -> %f\n\n", tmpcost, (*tmpvehiclei).totalcost + (*tmpvehiclej).totalcost);

					tmpnodel = l;
					tmpnodek = k;
					tmpcost = (*tmpvehiclei).totalcost + (*tmpvehiclej).totalcost;
				}
			}
		}
	}
	// if there is best swap
	if (tmpnodel != -1)
	{
		printf("Amoung all possible swaps, we choose the best: vehicle %d, position %d - vehicle %d, position %d\n",
			vehiclei.id, tmpnodek, vehiclej.id, tmpnodel);
		//printf("before\n");
		//printf("vehicle %d\n", i);
		//vehicle[i].vehicleroute.show();
		//printf("vehicle %d\n", j);
		//vehicle[j].vehicleroute.show();

		// store the cost before
		tmpcost = vehiclei.totalcost + vehiclej.totalcost;

		// store the intermediate node
		tmpclientk = vehiclei.route[tmpnodek];
		tmpclientl = vehiclej.route[tmpnodel];

		// make swap in position k, vehicle i
		vehiclei.route[tmpnodek] = tmpclientl;
		vehiclei.route[tmpnodek].next = tmpnodek + 1;
		// update load and cost
		vehiclei.update_dvrp();
		vehiclei.linkedlist2array();

		// make swap in position l, vehicle j
		vehiclej.route[tmpnodel] = tmpclientk;
		// maintain linking relation
		vehiclej.route[tmpnodel].next = tmpnodel + 1;
		// update load and cost
		vehiclej.update_dvrp();
		vehiclej.linkedlist2array();

		//printf("after:\n");
		//printf("vehicle %d\n", i);
		//vehicle[i].vehicleroute.show();
		//printf("vehicle %d\n", j);
		//vehicle[j].vehicleroute.show();
		//printf("cost reduce: %f -> %f\n", tmpcost, vehicle[i].vehicleroute.totalcost + vehicle[j].vehicleroute.totalcost);
	}
	else
	{
		// do no thing
		printf("no best swap!\n");
	}
	delete tmpvehiclei;
	delete tmpvehiclej;
}

// transformed from previous version
// -- GOOD
void Solution::perform_opt_dvrp()
{
	printf("\n\n\nperforming opt operator, DVRP ...\n");
	int i = 0, j = 0, k = 0;
	int jkstart = CLIENT_LIMIT;
	Vehicle tmpvehicle;
	int tmpoptj = -1, tmpoptk = -1;
	bool feasi_check_ret = false;
	double tmpcost;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		tmpoptj = -1, tmpoptk = -1;
		if (vehiclelist[i].len < 5) continue;
		tmpcost = vehiclelist[i].totalcost;
		// find out jstart and kstart
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			if (vehiclelist[i].route[j].cost > current_time)
			{
				jkstart = j + 1;
				break;
			}
		}

		// test all possible opts
		for (j = jkstart; j < vehiclelist[i].len - 2; j++)
		{
			for (k = jkstart; k < vehiclelist[i].len - 1; k++)
			{
				if (j == k || j + 1 == k || k + 1 == j) continue;
				tmpvehicle = vehiclelist[i];
				// TBC here 
				tmpvehicle.opt_operator_dvrp(j, k);
				// better: feasible and cost reduced
				if (tmpvehicle.totalcost < tmpcost && tmpvehicle.isfea_after_current == true)
				{
					// printf("a better opt solution!");
					// printf("vehicle %d, opt %d & %d\n", i, j, k);
					// tmpvehicle.show();
					tmpoptj = j;
					tmpoptk = k;
					tmpcost = tmpvehicle.totalcost;
				}
			}
		}
		// if there is best opt
		if (tmpoptj != -1)
		{
			printf("vehicle %d, choose the best opt: %d & %d. cost %lf -> %lf\n",
				i, tmpoptj, tmpoptk, vehiclelist[i].totalcost, tmpcost);
			vehiclelist[i].opt_operator_dvrp(tmpoptj, tmpoptk);
			// bestsolution.vehiclelist[i].show();	
		}
		else
		{
			printf("vehicle %d: No better opt found!\n", i);
		}
	}
	eval();
	printf("opt finished, update ...\n");
	// show();
}

// if a cancellation or negative variation, relax the demand of the nodes
// only nodes visited after current time are influenced, all routes influenced
// -- GOOD
void Solution::relax_demand_and_reload()
{
	int i;
	int j;
	int next_loc;
	double tmp_total_supply = 0;
	double tmp_total_demand = 0;
	double tmp_ratio = 0;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		next_loc = vehiclelist[i].next_visiting_loc(current_time);
		for (j = next_loc; j < vehiclelist[i].len; j++)
		{
			if (vehiclelist[i].route[j].type == PICK)
			{
				tmp_total_supply += vehiclelist[i].route[j].long_supply;
				tmp_total_supply += vehiclelist[i].route[j].prsb_supply;
			}
			else // delivery
			{
				tmp_total_demand += vehiclelist[i].route[j].demand;
			}
		}
	}
	// get ratio
	tmp_ratio = tmp_total_supply / tmp_total_demand;
	// reduce
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		next_loc = vehiclelist[i].next_visiting_loc(current_time);
		// reduce the demand after current time
		for (j = next_loc; j < vehiclelist[i].len; j++)
		{
			if (vehiclelist[i].route[j].type == DELV)
			{
				vehiclelist[i].route[j].demand = vehiclelist[i].route[j].init_demand * tmp_ratio;
				vehiclelist[i].route[j].reduced_demand = vehiclelist[i].route[j].demand * RELAX_RATIO;
			}
		}
	}
	// reload
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		vehiclelist[i].update_dvrp();
	}
	eval();
}

// based on current time, get next upcoming event
// based on the time of next upcoming event, get next current time
// collect events into event buffer along the way
double Solution::push_forward_time()
{
	double next_current_time = -1;
	char buff[512];
	printf("\n\n\n\n---- best solution push forward time!\n");
	printf("---- current time = %lf\n", current_time);

	// write to file
	sprintf(buff, "\n\n\n\n---- best solution push forward time!\n");
	(*pfdoutx) << buff;
	sprintf(buff, "---- current time = %lf\n", current_time);
	(*pfdoutx) << buff;

	Event *p = NULL;
	// based on current time, get next event
	// question: what if no next upcoming event?
	//           what if still events after routing is end?
	p = get_next_upcoming_event();

	// no next upcoming event, routing finished
	if (p == NULL)
	{
		printf("---- no next event, routing finished!\n");
		sprintf(buff, "---- no next event, routing finished!\n");
		(*pfdoutx) << buff;
		// return -1
		return -1;
	}

	printf("---- next upcoming event happens at time: %lf\n", p->time);
	p->show();
	sprintf(buff, "---- next upcoming event happens at time: %lf\n", p->time);
	(*pfdoutx) << buff;
	p->write((*pfdoutx));

	// still next upcoming event
	// check if this is a time based event and whether vehicles have finished
	if (vehicles_finished_routing(p))
	{
		// if vehicles have all finished, end routing
		// only in this case we may have unhandled events
		printf("all vehicles finished routing! return\n");
		sprintf(buff, "all vehicles finished routing! return\n");
		(*pfdoutx) << buff;
		return -1;
	}

	// pevent_buffer.push(p);

	// based on time of next event, let vehicles move to next upcoming node
	printf("---- vehicles move to current time!\n");
	sprintf(buff, "---- vehicles move to current time!\n");
	(*pfdoutx) << buff;
	next_current_time = vehicles_move_to_next_current(p->time);
	printf("---- next current time = %lf!\n", next_current_time);
	sprintf(buff, "---- next current time = %lf!\n", next_current_time);
	(*pfdoutx) << buff;
	current_time = next_current_time;
	return next_current_time;
}

// test if all vehicles have finished routing in this solution
bool Solution::vehicles_finished_routing(const Event *p)
{
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		// at least one vehicle has not finished yet
		if (vehiclelist[i].totalcost > p->time) return false;
	}
	return true;
}

// given event time, find out next current time
// for best solution
double Solution::vehicles_move_to_next_current(double event_time)
{
	double next_current = -1;
	double tmp_cost;
	Event *p = NULL;
	int i, j;

	// find out the furthest node after current event
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			// the first node of that route that is visited after that event
			// note that this if must happen
			if (vehiclelist[i].route[j].cost > event_time)
			{
				// if this node is further, record it as next current
				if (vehiclelist[i].route[j].cost > next_current)
				{
					next_current = vehiclelist[i].route[j].cost + EPSILON;
				}
				break;
			}
		}
	}

	// after previous loop, next current should be determined
	if (next_current == -1)
	{
		printf("bug detected!\n");
		system("pause");
	}

	// collect events between current_time and next_current
	// collect time based events
	for (i = 0; i < event_count_time; i++)
	{
		if (pevent_time[i]->time > current_time && pevent_time[i]->time < next_current)
		{
			pevent_buffer.push(pevent_time[i]);
			if (i != event_time_ptr)
			{
				printf("bug detected! event time ptr inconsistancy\n");
				system("pause");
			}
			event_time_ptr++;
		}
	}

	// collect node based events
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			tmp_cost = vehiclelist[i].route[j].cost;
			// if this node is visited between current and next current
			if (tmp_cost > current_time && tmp_cost < next_current)
			{
				p = is_event_node(vehiclelist[i].route[j]);
				if (p != NULL)
				{
					pevent_buffer.push(p);
				}
			}
		}
	}

	return next_current;
}

Event * Solution::is_event_node(Client node)
{
	int i;
	for (i = 0; i < event_count_node; i++)
	{
		if (pevent_node[i]->node_id == node.id && pevent_node[i]->ishandled == false)
		{
			pevent_node[i]->time = node.cost - EPSILON;
			return pevent_node[i];
		}
	}
	return NULL;
}

Event * Solution::get_next_event_node()
{
	Event *p_node = NULL;
	Event *tmp;
	double tmptime = TIME_LIMIT;
	int i, j;
	int next_loc;
	// from current location, get nearest event node
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		// find out the current location of the vehicle in its own route
		next_loc = -1;
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			if (vehiclelist[i].route[j].cost > current_time)
			{
				next_loc = j;
				break;
			}
		}
		// next_loc is the next node that the vehicle is about to visit
		if (next_loc != -1)
		{
			// next_loc is the next location/ node that is to be visited by this vehicle
			for (j = next_loc; j < vehiclelist[i].len - 1; j++)
			{
				tmp = is_event_node(vehiclelist[i].route[j]);
				// this node is a event object node
				if (tmp != NULL)
				{
					// make sure this event happens right before reaching that node in bestsolution
					// time of location based events are always adjusted dynamically with this function
					tmp->time = vehiclelist[i].route[j].cost - EPSILON;
					if (tmp->time < tmptime)
					{
						p_node = tmp;
						tmptime = tmp->time;
						break;
					}
				}
				// this node is not an event object
				// do nothing
				else
				{
					;
				}
			}
		}
		// have already passed all nodes
		// do nothing
		else
		{
			;
		}
	}
	return p_node;
}

Event * Solution::get_next_event_time()
{
	return pevent_time[event_time_ptr];
}

Event * Solution::get_next_upcoming_event()
{
	Event *p = NULL;
	Event *p_node = NULL;
	Event *p_time = NULL;
	Event *tmp = NULL;
	int i, j, next_loc;

	p_node = get_next_event_node();

	// after above, p_node will be pointing to the nearest upcoming event node, or be NULL
	// then get next time based event
	p_time = get_next_event_time();

	// bug detect
	if (p_time == NULL && event_time_ptr != event_count_time)
	{
		printf("bug detected! invalid time based event pointer!\n");
		system("pause");
	}

	// node based events finished
	if (p_node == NULL && p_time != NULL)
	{
		p = p_time;
		// event_time_ptr++;
	}
	// time based events finished
	else if (p_node != NULL && p_time == NULL)
	{
		p = p_node;
	}
	// all not finished 
	else if (p_node != NULL && p_time != NULL)
	{
		if (p_node->time < p_time->time)
		{
			p = p_node;
		}
		else
		{
			p = p_time;
			// event_time_ptr++;
		}
	}
	// routing finished
	else
	{
		p = NULL;
	}

	return p;
}

// from current time, to next current, collect events, update next current
void Solution::follow_next_current(double next_current)
{
	int i, j;
	Event *p;
	double tmp_cost; 
	char buff[512];

	printf("---- \n");
	sprintf(buff, "---- \n");
	(*pfdoutx) << buff;
	printf("---- good solution follow next current .... \n");
	sprintf(buff, "---- good solution follow next current .... \n");
	(*pfdoutx) << buff;
	printf("---- current time: %lf, next current: %lf\n", current_time, next_current);
	sprintf(buff, "---- current time: %lf, next current: %lf\n", current_time, next_current);
	(*pfdoutx) << buff;

	// want this vehicle to finish routing
	if (next_current == -1) next_current = TIME_LIMIT;

	// collect events between current_time and next_current
	// collect time based events
	for (i = 0; i < event_count_time; i++)
	{
		if (pevent_time[i]->time > current_time && pevent_time[i]->time < next_current)
		{
			pevent_buffer.push(pevent_time[i]);
			if (i != event_time_ptr)
			{
				printf("bug detected! event time ptr inconsistancy\n");
				system("pause");
			}
			event_time_ptr++;
		}
	}

	// collect node based events
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			tmp_cost = vehiclelist[i].route[j].cost;
			// if this node is visited between current and next current
			if (tmp_cost > current_time && tmp_cost < next_current)
			{
				p = is_event_node(vehiclelist[i].route[j]);
				if (p != NULL)
				{
					p->time = vehiclelist[i].route[j].cost - EPSILON;
					pevent_buffer.push(p);
				}
			}
		}
	}

	current_time = next_current;
	return;
}

//
Event * Solution::get_next_event()
{
	Event *p = NULL;
	if (pevent_buffer.empty() == false)
	{
		p = pevent_buffer.top();
		pevent_buffer.pop();
	}
	return p;
}

// only handler event, do not touch timing system
// -- DONE
void Solution::batch_event_handling()
{
	char buff[512];
	clock_t tmp_time;
	printf("\n\n---- start batch event handling!\n");
	printf("---- %d events in total!\n\n", pevent_buffer.size());
	sprintf(buff, "\n\n---- start batch event handling!\n");
	(*pfdoutx) << buff;
	sprintf(buff, "---- %d events in total!\n\n", pevent_buffer.size());
	(*pfdoutx) << buff;

	Event *p = NULL;
	p = get_next_event();
	while (p != NULL)
	{
		p->show();

		p->write(*pfdoutx);
		sprintf(buff, "---- solution before this event:\n");
		(*pfdoutx) << buff;
		write(*pfdoutx);
		
		tmp_time = clock();

		// debug:
		if (p->ishandled == true)
		{
			printf("bug detected: an event already handled, please check timing system again!\n");
			system("pause");
		}
		if (p->type == NA)
		{
			NodeArrival *pna = (NodeArrival *)p;
			if (REACTION_TYPE == DYNAMIC_HANDLER)
			{
				NA_handler_dynamic(*pna);
			}
			else
			{
				NA_handler_static(*pna);
			}
		}
		else if (p->type == NC)
		{
			NodeCancellation *pnc = (NodeCancellation *)p;
			if (REACTION_TYPE == DYNAMIC_HANDLER)
			{
				NC_handler_dynamic(*pnc);
			}
			else
			{
				NC_handler_static(*pnc);
			}
		}
		else if (p->type == NV)
		{
			NegativeVariation *pnv = (NegativeVariation *)p;
			if (REACTION_TYPE == DYNAMIC_HANDLER)
			{
				NV_handler_dynamic(*pnv);
			}
			else
			{
				NV_handler_static(*pnv);
			}
		}
		else if (p->type == PV)
		{
			PositiveVariation *ppv = (PositiveVariation *)p;
			if (REACTION_TYPE == DYNAMIC_HANDLER)
			{
				PV_handler_dynamic(*ppv);
			}
			else
			{
				PV_handler_static(*ppv);
			}
		}
		else
		{
			printf("bug detected here: invalid event type!\n");
			system("pause");
		}

		tmp_time = clock() - tmp_time;

		sprintf(buff, "\n\n---- solution after this event:\n");
		(*pfdoutx) << buff;
		write(*pfdoutx);
		sprintf(buff, "---- computational time cost: %lf\n\n\n", ((double)tmp_time) / CLOCKS_PER_SEC);
		(*pfdoutx) << buff;

		double avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery;
		double tmpwaste;
		get_fillrate(avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery);
		get_waste(tmpwaste);
		info_list[info_count].set_value( ( (double)tmp_time ) / CLOCKS_PER_SEC , solutioncost, avg_fillrate_delivery,
			min_fillrate_delivery, max_fillrate_delivery, tmpwaste, p->type, p->time, p->node_id);
		info_count++;
		p->ishandled = true;
		p = get_next_event();
	}

	return;
}

void Solution::get_fillrate(double &avg_fillrate_delivery,
	double &min_fillrate_delivery, double &max_fillrate_delivery)
{
	double total_fill_rate = 0;
	int total_cnt = 0;
	max_fillrate_delivery = 0;
	min_fillrate_delivery = 1;
	int i, j;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		for (j = 1; j < vehiclelist[i].len - 1; j++)
		{
			if (vehiclelist[i].route[j].type == DELV)
			{
				total_fill_rate += vehiclelist[i].route[j].fillrate;
				total_cnt += 1;
				if (vehiclelist[i].route[j].fillrate > max_fillrate_delivery)
				{
					max_fillrate_delivery = vehiclelist[i].route[j].fillrate;
				}
				if (vehiclelist[i].route[j].fillrate < min_fillrate_delivery)
				{
					min_fillrate_delivery = vehiclelist[i].route[j].fillrate;
				}
			}
		}
	}
	avg_fillrate_delivery = total_fill_rate / total_cnt;
	return;
}

void Solution::get_waste(double &tmpwaste)
{
	tmpwaste = 0;
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		tmpwaste += vehiclelist[i].prsb_finalload; 
	}
	return;
}

void Solution::write_info(ofstream &fdoutx)
{
	int i;
	char buff[512];
	sprintf(buff, " event_type, event_time, node_id, time_taken, cost\n  avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery, waste\n");
	fdoutx << buff;
	for (i = 0; i < info_count; i++)
	{
		info_list[i].write(fdoutx);
	}
}

void AdditionalInfo::write(ofstream &fdoutx)
{
	char buff[512];
	sprintf(buff, "%d, %lf, %d, %lf, %lf\n  %lf, %lf, %lf, %lf\n", event_type, event_time,
		node_id, time_taken, cost, avg_fillrate_delivery, min_fillrate_delivery,
		max_fillrate_delivery, waste);
	fdoutx << buff;
}

// -- GOOD
void Solution::change_supply(Event *p)
{
	printf("---- change supply on event!\n");
	int i, j;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			if (vehiclelist[i].route[j].id == p->node_id)
			{
				vehiclelist[i].route[j].change_supply(p);
				vehiclelist[i].update_dvrp();
				goto L1;
			}
		}
	}
L1:
	eval();
	return;
}

int Solution::find_event_vehicle(int node_id)
{
	int i;
	int j;
	int vehicle_id = -1;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		for (j = 0; j < vehiclelist[i].len; j++)
		{
			if (vehiclelist[i].route[j].id == node_id)
			{
				vehicle_id = i;
				return vehicle_id;
			}
		}
	}
	// control reaches here, no vehicle found
	// debug here
	printf("error: no vehicle corresponding to that event!\n");
	system("pause");
	return -1;
}

// ---- handlers ----
// dynamic
// -- DONE
void Solution::NA_handler_dynamic(NodeArrival &na_event)
{
	bool tmp_ret;
	// simply insert it
	tmp_ret = insert_event_na(na_event);
	if (tmp_ret == false)
	{
		na_event.successfully_handled = false;
		na_event.show();
	}
}

// *************************************************************************
// Discussion: feasibility
// When comparing good solutions with best solution, we need to decide which 
//   one is "better" feasibility is a criterion to determine better solution.
// When considering if a operator makes a solution better, we still need 
//   to consider if this operator makes this solution feasible.
// We compare good solutions with best solution at every current time in our 
//   timing system, and we may perform operators at every current time.
// A solution may be infeasible before a particular current time
//   but may still be feasible after that time.
// when comparing two solutions, in terms of feasibility, we make the following
//   change: feasibility -> dvrp-feasibility
// dvrp-feasibility = feasible after current time
// *************************************************************************

// still visit that node but load nothing
// when this handler is called, we have already passed that node 
// -- DONE
void Solution::NC_handler_dynamic(const NodeCancellation &nc_event)
{
	// change the supply to be 0, but still visit that node
	change_supply((Event *) &nc_event);
	// if feasible, end event handling
	if (isfea_after_current)
	{
		printf("after node cancellation, still feasible!\n");
		return;
	}
	// else, relax the demand after current time, and use operator chain
	else
	{
		// note: even if demand is relaxed, this solution may still not dvrp-feasible
		printf("after node cancellation, not feasible anymore!\n");
		printf("relax demand and reload .... \n");
		relax_demand_and_reload();
		// note: even if the operator chain is performed, this solution may still not dvrp-feasible
		//   but we will accept it anyway
		printf("perform operator chain .... \n");
		operator_chain();
	}
}

// Note: this event is handled exactly the same as node cancellation
// -- DONE
void Solution::NV_handler_dynamic(const NegativeVariation &nv_event)
{
	// change the supply
	change_supply((Event *) &nv_event);
	// if feasible, end event handling
	// if feasible, end event handling
	if (isfea_after_current)
	{
		printf("after negative variation, still feasible!\n");
		return;
	}
	// else, relax the demand after current time, and use operator chain
	else
	{
		// note: even if demand is relaxed, this solution may still not dvrp-feasible
		printf("after node cancellation, not feasible anymore!\n");
		printf("relax demand and reload .... \n");
		relax_demand_and_reload();
		// note: even if the operator chain is performed, this solution may still not dvrp-feasible
		//   but we will accept it anyway
		printf("perform operator chain .... \n");
		operator_chain();
	}
}

// -- DONE
void Solution::PV_handler_dynamic(const PositiveVariation &pv_event)
{
	int event_vehicle_id;
	// reallocate the route
	change_supply((Event *) &pv_event);
	// question: do we need to restore the need of supply to previous state before relaxation?
	// - for now I do not do that.
	event_vehicle_id = find_event_vehicle(pv_event.node_id);
	vehiclelist[event_vehicle_id].waste_distribute_after_current();
	// note: this handler may still lead to dvrp-infeasible
}

// collect nodes that have not been visited from current time
void Solution::collect_nodes()
{
	dvrp_node_count = 0;
	int i;
	int j;
	int next_loc;
	int tmpcnt;
	printf("collecting nodes .... \n");
	printf("current time: %lf\n", current_time);
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		next_loc = vehiclelist[i].next_visiting_loc(current_time);
		// already finished
		if (next_loc == -1) continue;
		// node cannot be the tail
		printf("vehicle %d, next visiting node at location: %d, time %lf\n",
			i, next_loc, vehiclelist[i].route[next_loc].cost);
		tmpcnt = 0;
		for (j = next_loc + 1; j <= vehiclelist[i].len - 2; j++)
		{
			add_into_node_set(vehiclelist[i].route[j]);
			tmpcnt++;
		}
		printf("vehicle %d, len = %d, %d nodes have not been visited\n", i, vehiclelist[i].len, tmpcnt);
	}
	// debug:
	printf("%d nodes have not visited!\n", dvrp_node_count);
	return;
}

// add a node into node set for later route reconstruction
void Solution::add_into_node_set(Client node)
{
	node.isinserted = false;
	node.load = 0;
	node.long_load = 0;
	node.prsb_load = 0;
	node.long_delivered = 0;
	node.prsb_delivered = 0;
	node.fillrate = 0;
	node.cost = 0;
	node.next = -1;
	node.distance = 0;
	dvrp_node_set[dvrp_node_count] = node;
	dvrp_node_count++;
}

void Solution::calculate_equity()
{
	int i;
	int tmp_vehicle_in_use = 0;
	equity = 0;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		tmp_vehicle_in_use++;
		vehiclelist[i].calculate_equity();
		equity += vehiclelist[i].equity;
	}
	equity /= tmp_vehicle_in_use;
	return;
}

// modified from previous version
// -- DONE
void Solution::rebuild()
{
	printf("rebuilding the solution .... \n");
	double constructioncost = 0;
	int i, j, l;
	int jumpcnt;
	unserved_dvrp = false;

	int tmpposition;
	double tmpmincost;
	double tmpcost;
	bool terminateflag;
	int feasibilityret;
	int next_loc;

	Vehicle *tmpvehicle = new Vehicle;

	// Initialize the node set
	for (i = 0; i < dvrp_node_count; i++)
	{
		dvrp_node_set[i].isinserted = false;
	}

	// sort the clients according to its location
	qsort(dvrp_node_set, dvrp_node_count, sizeof(Client), clientcmp);

	// for every vehicles on the way
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		next_loc = vehiclelist[i].next_visiting_loc(current_time);
		if (next_loc == -1) continue;
		if (next_loc >= vehiclelist[i].len - 1) continue;
		
		// initiate the vehicle
		// directly link the next_loc and the tail, then nodes will later be inserted in between
		vehiclelist[i].route[next_loc].next = vehiclelist[i].tail;
		vehiclelist[i].len = next_loc + 2;
		vehiclelist[i].linkedlist2array();
		vehiclelist[i].update_dvrp();
		
		*tmpvehicle = vehiclelist[i];
		jumpcnt = 0;

		// try every node that has not been inserted
		for (j = 0; j < dvrp_node_count; j++)
		{
			// to insert it in every possible route location in vehicle[i]'s route
			// terminal condition: 1> no avaiable node 2> vehicle time limit use up
			if (dvrp_node_set[j].isinserted == true) continue;

			// initialize insert attempt
			tmpposition = -1;
			// the maxmum of the data type float 
			tmpmincost = numeric_limits<double>::max();
			// use this flag to detect termination
			terminateflag = false;

			// l runs through every possible position in the route of a vehicle
			for (l = next_loc; l < vehiclelist[i].len - 1; l++)
			{
				// printf("Trying node %d at vehicle %d, position %d\n", client[j].id, i, l);
				// feasibilityret = tmpsolution.vehiclelist[i].feasibility_check(j, k); // -- ok
				// here are the return value of feasibility_check function:
				// 0: feasible; 1: violate feasibility criteria 1,2,3; -1: violate feasibility criteria4
				// printf("trying insertion...\n");
				*tmpvehicle = vehiclelist[i];
				(*tmpvehicle).insert_sdvrp(dvrp_node_set[j], l);

				// feasible after next visiting node
				if ((*tmpvehicle).isfea == true)
				{
					// check if the cost will be reduced
					tmpcost = (*tmpvehicle).totalcost;
					if (tmpcost < tmpmincost)
					{
						// make a temporary record
						tmpmincost = tmpcost;
						tmpposition = l;
					}
				}

				else // isfea == false 
				{
					// terminated
					if ( (*tmpvehicle).route[(*tmpvehicle).tail].cost > (*tmpvehicle).closetime )
					{
						// terminate
						printf("vehicle %d terninated!\n", i);
						terminateflag = true;
						// terminate detection codes
						// printf("terminate detected!\n");
						// vehiclelist[i].show();
						// printf("node and position failed to insert: %d, %d", j, k);
						break;
					}
					else // violate feasibility criteria 1,2,3
					{
						// try next position
						// do nothing
						;
					}
				}
			}
			if (terminateflag == true)
			{
				break;
			}
			if (tmpposition == -1)
			{
				// no feasible position, try next node
				jumpcnt++;
				// jump detection codes
				// tmpsolution.vehiclelist[i].show();
				// printf("node %d jumped\n", scheduletable[currentday].clienttable[j].id);
			}
			else
			{
				// insert and update
				vehiclelist[i].insert_sdvrp(dvrp_node_set[j], tmpposition);
				// printf("vehicle %d insert node %d at position %d, cost = %lf\n", 
					// i, dvrp_node_set[j].id, tmpposition, vehiclelist[i].totalcost);
				dvrp_node_set[j].isinserted = true;

				if (jumpcnt > 0)
				{
					j = 0;
					jumpcnt = 0;
				}
			}
		}

		// try out all node or terminated by feasibility criteria4
		printf("final route of vehicle %d:\n", i);
		vehiclelist[i].show();
		constructioncost += vehiclelist[i].totalcost;
	}

	unservedclient = 0;
	printf("Detecting unserved clients...\n");

	// if unserved, will insert them infeasibly after re-routing
	for (i = 0; i < dvrp_node_count; i++)
	{
		if (dvrp_node_set[i].isinserted == false)
		{
			printf("client %d did not served!\n", dvrp_node_set[i].id);
			unserved_dvrp = true;
		}
	}

	eval();

	// route construction finished
	printf("Route re-construction finished, store as bestsolution.\n");
	// bestsolution.show();

	delete tmpvehicle;
	return;
}

// set shift table after current time
// -- DONE
void Solution::set_sttable_after_current()
{
	int i, j, k, l, m;
	int next_loc;
	onevehicle = true;
	stp = 0;
	m = 0;
	printf("Setting sttable...\n");
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		for (j = 1; j <= Vehicle::totalvehicle; j++)
		{
			if (vehiclelist[j].len <= 2 || j == i) continue;

			next_loc = vehiclelist[i].next_visiting_loc(current_time);
			if (next_loc == -1) next_loc = CLIENT_LIMIT;
			k = next_loc + 1;
			
			for (; k <= vehiclelist[i].len - 2; k++)
			{
				onevehicle = false;

				next_loc = vehiclelist[j].next_visiting_loc(current_time);
				if (next_loc == -1) next_loc = CLIENT_LIMIT;
				l = next_loc;

				for (; l <= vehiclelist[j].len - 2; l++)
				{
					sttable[m].vehiclefromid = i;
					sttable[m].vehicletoid = j;
					sttable[m].nodepos = k;
					sttable[m].nodeid = vehiclelist[i].route[k].id;
					sttable[m].position = l;
					sttable[m].isvalid = true;
					m++;
				}
			}
		}
	}
	printf("Set sttable, length of sttable: %d\n", m);
	possibilities = m;
	return;
}

// check feasibility, sdvrp version
// if not better, then always tabu solution
// -- DONE
int Solution::feasibility_check_sdvrp()
{
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		if (vehiclelist[i].feasibility_check_sdvrp() != 0) return 1;
	}
	// Bug detecting code
	if (solutioncost != evalfunc)
	{
		printf("A illegal route!\n");
		show();
		while (1);
		exit(1);
	}
	return 0;
}

// -- DONE
void Solution::tabu_search_after_current()
{
	printf("start to perform tabu search .... \n");

	int i, j, k, v, w;
	int feasibilityret;
	int total_fea = 0;
	int total_infea = 0;
	int total_tabu = 0;
	int tabu_cnt = 0;
	int tabu_total_cnt = 0;
	int neighborfrom;
	int neighborto;
	int neighbornode;
	int neighborpos;
	int next_loc;
	double neighbormf;
	double tabucost;
	bool feasibilityret_tabu;
	Shiftrecord shiftobj;

	Solution *pbestsolution = new Solution;
	Solution *ptmpsolution = new Solution;
	Solution *pcurrentsolution = new Solution;

	printf("\n\n\nStart tabu search...\n");
	// printf("Initializing...\n");

	// printf("Initial best solution:\n");
	eval();
	set_sttable_after_current();
	// show();

	*pcurrentsolution = *this;
	*pbestsolution = *this;
	// currentsolution.show();

	// start the search algorithm, stop criteria check
	tabu_para_reset();
	tahistory.reset();
	for (i = 0; i < STOPCRITERIA; i++)
	{
	stageii:
		if ((*pbestsolution).evalfunc == 0)
		{
			printf("best solution bug detected!\n");
			printf("i = %d\n");
			(*pcurrentsolution).show();
			system("pause");
		}
		*ptmpsolution = *pcurrentsolution;
		// Check stop criteria
		if (i >= STOPCRITERIA)
		{
			printf("Hit stop criteria at mediate! return.\n");
			break;
		}

		// Apply the shift operator
		// Note that member function choose_shift will do four things
		// 1> randomly choose a shift
		// Note that we have two restrictions for the choice
		// First, the chioce should not be in the tabu history.
		// To ensure this, we have a tabu history table, which is implemented by the class tabu history
		// Second, we should not choose the operation we have performed before.
		// To ensure this, we have a operation history list in che class solution
		// 2> Update the current solution operation history
		// 3> If this operation is tabu, we will update tabu history. We will leave this for the later tabu check procedure.
		// 4> If we have checked all the shift option, can not find a better or tabu, then it will return a unvalid mark.
		//    In this case we will terminate the iteration.
		// This function will return a class chiftrecord containing all the information we need to perform a shift
		shiftobj = (*pcurrentsolution).choose_shift_st(tahistory);
		if (shiftobj.isvalid == false)
		{
			printf("No valid shift!\n");
			break;
		}
		//printf("choose shift...\n");


		// Caution: This function will make sure the following situation does not happen:
		//          After tabu search, no better solution found, then the current solution roll back to best solution,
		//			but we may choose the same shift operation as before.
		// To prevent this, we compare current solution with the best solution
		// If they are the same, means tabu search reaches tenure, roll back to current solution
		// bool tmpcmp = solution_compare(currentsolution, bestsolution);
		// Then we should insert the operation to the history
		// if(tmpcmp == true) bestsolution.insert_operation_record(shiftobj);

		// make the shift, maintain the linkedlist structure, update all the parameters in every route
		// If the shift operation actually delete a route, then modify the varible vehicleinuse

		/*printf("Before shift\n");
		(*ptmpsolution).show();
		printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
		  shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.nodepos, shiftobj.vehicletoid, shiftobj.position);*/

		shift_operator_sdvrp(*ptmpsolution, shiftobj);

		// use eval() to evaluate the value: evalfunc, mevalfunc and solutioncost
		// note that when a solution is feasible, we will have: evalfunc = solutioncost
		(*ptmpsolution).eval();
		// printf("After eval\n");
		// (*ptmpsolution).show();

		// Check the feasibility.
		// note that this isfea is applicable only after current time
		feasibilityret = (*ptmpsolution).feasibility_check_sdvrp();
		if (feasibilityret == 0)
		{
			total_fea++;
			// printf("feasible, tmp cost: %lf, best cost: %lf\n", (*ptmpsolution).solutioncost, (*pbestsolution).solutioncost);
		}
		else
		{
			total_infea++;
			// printf("infeasible, tmp eval: %lf, best eval: %lf\n", (*ptmpsolution).evalfunc, (*pbestsolution).evalfunc);
		}
		// if (i == 500) system("pause");

		// If a better solution
		if ((feasibilityret == 0) && ((*ptmpsolution).evalfunc < (*pbestsolution).evalfunc))
		{

			printf("\n\n Find a better solution!i = %d\n", i);
			printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
				shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.nodepos, shiftobj.vehicletoid, shiftobj.position);

			printf("best solution: Totalcost = %.lf, evalfunc = %.lf\n", 
				 (*pbestsolution).solutioncost, (*pbestsolution).evalfunc);
			printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", 
				 (*ptmpsolution).solutioncost, (*ptmpsolution).evalfunc);
			// printf("Before:\n");
			// bestsolution.show();
			// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);


			// Set the linked list to array
			// (*ptmpsolution).vehiclelist[shiftobj.vehiclefromid].linkedlist2array();
			// (*ptmpsolution).vehiclelist[shiftobj.vehicletoid].linkedlist2array();
			// tmpsolution.show();
			// printf("total vehicle %d\n", Vehicle::totalvehicle);

			// Update the operation history
			// bestsolution.update();
			// tmpsolution.update();

			// Set the best solution
			printf("Find a better solution! Update to best solution!\n");
			// solution_copy(tmpsolution, bestsolution)

			*pbestsolution = *ptmpsolution;


			// printf("After:\n");
			// (*pbestsolution).show();
			// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);


			// Reset tabu history
			tahistory.reset();

			// Reset sequential tabu table
			(*pbestsolution).set_sttable_after_current();

			// Prepare the current solution
			// currentsolution.update();
			// solution_copy(bestsolution, currentsolution);
			(*pcurrentsolution) = (*pbestsolution);
			tabu_para_reset();
		}
		else
		{
			// If a tabu solution, then begin/continue tabu search.
			// In our algorithm, we will treat the start of a new tabu search and continue an existing tabu search as the same.
			// Just the level of depth of tabu search is different
			if (feasibilityret == 1 && (*ptmpsolution).evalfunc < (*pcurrentsolution).evalfunc)
			{
				printf("\n\nFind a tabu solution!i = %d\n", i);
				printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
					shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.nodepos, shiftobj.vehicletoid, shiftobj.position);
				printf("current solution: Totalcost = %.lf, evalfunc = %.lf\n", 
					(*pcurrentsolution).solutioncost, (*pcurrentsolution).evalfunc);
				printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", 
					(*ptmpsolution).solutioncost, (*ptmpsolution).evalfunc);
				printf("Tabu depth: %d\n", tabu_cnt);
				// set the current solution
				total_tabu++;
				// currentsolution.update();
				// solution_copy(tmpsolution, currentsolution);
				*pcurrentsolution = *ptmpsolution;

				// Start/continue tabu search
				// If tabu_cnt == 0, then we start a new tabu search. If tabu_cnt > 1, means we are continuing an axisting rabu search


				// If the tabu search depth is 0, means we just find a tabu solution from best solution.
				// We will remember the sequantial tabu table in case we reach the bottom of tabu search, cannot find a inspired solution, roll back to bestsolution
				if (tabu_cnt == 0)
				{
					copy_sttable(*pcurrentsolution, *pbestsolution);
				}
				tabu_cnt++;

				// Tabu search reaches to the depth limit
				if (tabu_cnt == TABUTENURE)
				{
					// Roll back to last best solution
					printf("Tabu limit exceeded, roll back to former best solution!\n\n\n");
					// currentsolution.update();
					// solution_copy(bestsolution, currentsolution);
					*pcurrentsolution = *pbestsolution;
					tabu_cnt = 0;

					// Clean the tabu history
					tahistory.reset();

					// Go back to the beginning of iteration
					tabu_para_reset();
					continue;
				}

				// Push a new record
				printf("Putting this into tabu history...\n");
				tahistory.push(shiftobj);

				// Update the retaining time of all the retained node
				tahistory.update();

				// Initialize neighbors generating
				neighborfrom = -1;
				neighbormf = numeric_limits<double>::max();

				// Generate all the neighbors
				// For every vehicle j in current solution
				// Note that the vehicle index starts from 1
				printf("Generating all the neighbors...\n");

				// printf("total vehicle %d\n", Vehicle::totalvehicle);
				for (j = 1; j <= Vehicle::totalvehicle; j++)
				{
					// and vehicle k in current solution
					if ((*pcurrentsolution).vehiclelist[j].len <= 2) continue;
					for (k = 1; k < Vehicle::totalvehicle; k++)
					{
						if (j == k || (*pcurrentsolution).vehiclelist[k].len <= 2) continue;
						// for every node v in route of vehicle j
						next_loc = (*pcurrentsolution).vehiclelist[j].next_visiting_loc(current_time);
						if (next_loc == -1) next_loc = CLIENT_LIMIT;
						v = next_loc + 1;
						for (; v < (*pcurrentsolution).vehiclelist[j].len - 1; v++)
						{
							// and every possible position w in vehicle k
							// note this iteration is based on array structure
							next_loc = (*pcurrentsolution).vehiclelist[k].next_visiting_loc(current_time);
							if (next_loc == -1) next_loc = CLIENT_LIMIT;
							w = next_loc;
							for (; w < (*pcurrentsolution).vehiclelist[k].len - 1; w++)
							{
								// if not in history record
								Shiftrecord tmpsrt;
								tmpsrt.vehiclefromid = j;
								tmpsrt.vehicletoid = k;
								tmpsrt.nodepos = v;
								tmpsrt.nodeid = (*pcurrentsolution).vehiclelist[j].route[v].id;
								tmpsrt.position = w;
								// printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
								// tmpsrt.nodeid, tmpsrt.vehiclefromid, tmpsrt.nodepos, tmpsrt.vehicletoid, tmpsrt.position);
								if (tahistory.check_record(tmpsrt) == false)
								{
									// tmpsolution.update();
									// solution_copy(currentsolution, tmpsolution);
									(*ptmpsolution) = (*pcurrentsolution);
									shift_operator_sdvrp((*ptmpsolution), tmpsrt);
									(*ptmpsolution).eval();
									feasibilityret_tabu = (*ptmpsolution).feasibility_check_sdvrp();

									// if aspired, i.e. feasible and cost reduced
									if ((feasibilityret_tabu == 0) && ((*ptmpsolution).solutioncost < (*pbestsolution).solutioncost))
									{
										printf("\n\nFind an aspired solution from neighbours. i = %d\n", i);
										printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
											tmpsrt.nodeid, tmpsrt.vehiclefromid, tmpsrt.nodepos, tmpsrt.vehicletoid, tmpsrt.position);
										printf("best solution: Totalcost = %.lf, evalfunc = %.lf\n", 
											(*pbestsolution).solutioncost, (*pbestsolution).evalfunc);
										printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", 
											(*ptmpsolution).solutioncost, (*ptmpsolution).evalfunc);
										// printf("Before:\n");
										// bestsolution.show();
										// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);

										// Set the linked list to array
										// (*ptmpsolution).vehiclelist[tmpsrt.vehiclefromid].linkedlist2array();
										// (*ptmpsolution).vehiclelist[tmpsrt.vehicletoid].linkedlist2array();

										// Set the operation history
										// bestsolution.update();
										// tmpsolution.update();

										// Copy to best solution
										// solution_copy(tmpsolution, bestsolution);
										*pbestsolution = *ptmpsolution;
										(*pbestsolution).set_sttable_after_current();

										// printf("After:\n");
										// (*pbestsolution).show();
										// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);

										// currentsolution.update();

										// Set the currentsolution
										// solution_copy(bestsolution, currentsolution);
										(*pcurrentsolution) = (*pbestsolution);

										// Reset tabu search counter
										tabu_cnt = 0;

										// Reset tabu history
										printf("reset tabu history\n");
										tahistory.reset();

										// note that here we use goto to get back to the beginning of iteration
										tabu_total_cnt++;
										i++;
										tabu_para_reset();
										goto stageii;
									}
									// if not inspired
									else
									{
										// if a better neighbor
										if ((*ptmpsolution).mevalfunc < neighbormf)
										{
											// record the new better neighbor
											// printf("A better neighbour! mf: %f -> %f, neighborfrom %d\n", tmpsolution.mevalfunc, neighbormf, j);
											neighborfrom = j;
											neighborto = k;
											neighbornode = v;
											neighborpos = w;
											neighbormf = (*ptmpsolution).mevalfunc;
										}
									}
								}
								else
								{
									// printf("shift in record, try next shift...\n");
								}
							}
						}
					}
				}
				// Not even a best neighbour
				if (neighborfrom == -1)
				{
					printf("There is no best neighbour, roll back to last best solution!\n");
					*pcurrentsolution = *pbestsolution;
					tabu_cnt = 0;

					// Clean the tabu history
					tahistory.reset();

					// Go back to the beginning of iteration
					tabu_para_reset();
					(*pcurrentsolution).choose_shift_st(tahistory);
					continue;
				}
				// There is a best neighour.
				// Update the current sulotion to the best neighbor
				printf("\n\nNo aspired solution! choose the best neighobor to be the current solution! i = %d\n", i);
				// printf("Shift node%d from vehicle%d to vehicle%d", currentsolution.vehiclelist[neighborfrom].vehicleroute.routelist[neighbornode].id,
				// neighborfrom, neighborto);
				Shiftrecord tmpsrd;
				tmpsrd.vehiclefromid = neighborfrom;
				tmpsrd.vehicletoid = neighborto;
				tmpsrd.nodepos = neighbornode;
				tmpsrd.position = neighborpos;
				tmpsrd.nodeid = (*pcurrentsolution).vehiclelist[neighborfrom].route[neighbornode].id;
				printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
					tmpsrd.nodeid, tmpsrd.vehiclefromid, tmpsrd.nodepos, tmpsrd.vehicletoid, tmpsrd.position);
				shift_operator_sdvrp((*pcurrentsolution), tmpsrd);
				// Evaluate the solution
				(*pcurrentsolution).eval();

				// Update the operation history
				// currentsolution.update();

				// Reset the sttable
				(*pcurrentsolution).set_sttable_after_current();
				//currentsolution.show();


				// update alpha, beta, gamma
				feasibilityret = (*pcurrentsolution).feasibility_check_sdvrp();
				// If feasible
				tabu_para_update(!feasibilityret);
			}
			// else: no tabu, do nothing
			// if(feasibilityret == 0) printf("This solution is feasible but not better\n");
			// else printf("The solution is infeasible but not tabu!\n");
			// printf("current solution: Totalcost = %.lf, evalfunc = %.lf\n", currentsolution.solutioncost, currentsolution.evalfunc);
			// printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", tmpsolution.solutioncost, tmpsolution.evalfunc);
			// printf("Try next\n\n\n");
			// currentsolution.show_history();
		}
		tabu_total_cnt++;
		if (i == STOPCRITERIA - 1) printf("Hit stop criteria! return.\n");
	}

	printf("tabu finished. cost: %lf\n", (*pbestsolution).solutioncost);
	printf("After tabusearch, our final best solution is: \n");
	(*pbestsolution).show();
	tabucost = (*pbestsolution).solutioncost;

	printf("total tabu circle:%d\ttotal fea:%d\ttotal infea:%d\ttotal tabu:%d\n", 
		tabu_total_cnt, total_fea, total_infea, total_tabu);

	delete pbestsolution;
	delete ptmpsolution;
	delete pcurrentsolution;
	return;
}

// the same as DVRP, allow infeasibility
void Solution::opt_after_current()
{
	perform_opt_dvrp();
}

void Solution::waste_distribute_after_current()
{
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		vehiclelist[i].waste_distribute_after_current();
	}
}

// perform routing
// just change the start location of previous nodes
// change previous besttolusion to be tmpbestsolution
// do not need to maintain good solutions
void Solution::route_from_current()
{
	// tabu search after current
	tabu_search_after_current();
	// opt after current
	opt_after_current();
	// waste distribution after current
	waste_distribute_after_current();
}

// the same as node insert dvrp
// only insert the node into routes that have not finished
void Solution::insert_node_sdvrp(Client node)
{
	int i, j;
	int next_loc;
	NodeInsertRecord tmp_insert;
	NodeInsertRecord best_insert;
	Vehicle *tmpvehicle = new Vehicle();

	Client tmpclient = node;
	bool tmpret;
	bool vehicles_all_finished;

	tmp_insert.valid = false;
	best_insert.valid = false;
	best_insert.cost_increase = TIME_LIMIT;

	vehicles_all_finished = true;

	// for every route in this solution
	for (i = 1; i < Vehicle::totalvehicle; i++)
	{
		if (vehiclelist[i].len <= 2) continue;
		// find out the next node to be visit 
		next_loc = vehiclelist[i].next_visiting_loc(current_time);

		if (next_loc == -1 || next_loc == vehiclelist[i].len - 1)
		{
			printf("vehicle %d is finished, cannot insert!\n", i);
			continue;
		}
		else
		{
			vehicles_all_finished = false;
		}

		// still possible insertion locations
		// (*tmpvehicle) = vehiclelist[i];

		// from this node, try all insertions, find the best 
		for (j = next_loc; j < vehiclelist[i].len - 1; j++)
		{
			*tmpvehicle = vehiclelist[i];
			tmp_insert.vehicleid = i;
			tmp_insert.position = j;

			(*tmpvehicle).insert_na(tmpclient, tmp_insert.position);
			// need to modify: if this vehicle is feasible after this current time
			tmp_insert.valid = (*tmpvehicle).isfea_after_current;
			tmp_insert.cost_increase = (*tmpvehicle).totalcost - vehiclelist[i].totalcost;

			// the valid bit is correcponding to feasibility
			if (tmp_insert.valid == true && best_insert.valid == false)
			{
				best_insert = tmp_insert;
			}
			else if (tmp_insert.valid == true && best_insert.valid == true)
			{
				// a better insert with lower cost increase
				if (tmp_insert.cost_increase < best_insert.cost_increase)
				{
					best_insert = tmp_insert;
				}
			}
			else if (tmp_insert.valid == false && best_insert.valid == false) // tmp_insert is not valid
			{
				if (tmp_insert.cost_increase < best_insert.cost_increase)
				{
					best_insert = tmp_insert;
				}
			}
			else // tmp insert not valid, best insert is valid
			{
				;
			}
		}
	}

	// after previous procedure, we may have a best insert or not
	// if not feasible, construct a new route

	// for test, make it not feasible on purpose
	// best_insert.valid = false
	if (best_insert.valid == false)
	{
		// must have a best insert
		if (best_insert.cost_increase == TIME_LIMIT)
		{
			printf("did not find best insert! vehicles all finished = %d\n", vehicles_all_finished);
			delete tmpvehicle;
			return;
			// do nothing
			goto END_INSERT;
			system("pause");
		}
		// -- TESTED
		printf("best insert is not valid, use operator chain\n");
		printf("insert this new node in vehicle %d, position %d\n", best_insert.vehicleid, best_insert.position);
		vehiclelist[best_insert.vehicleid].insert_na(tmpclient, best_insert.position);
		operator_chain();
	}
	// we have best insertion
	else
	{
		printf("best insert is valid!\n");
		printf("insert this new node in vehicle %d, position %d\n", best_insert.vehicleid, best_insert.position);
		// vehiclelist[best_insert.vehicleid].show();
		vehiclelist[best_insert.vehicleid].insert_na(tmpclient, best_insert.position);
		// vehiclelist[best_insert.vehicleid].show();
		eval();
	}
END_INSERT:
	delete tmpvehicle;
	return;
}

void Solution::insert_unserved_nodes()
{
	if (unserved_dvrp == false) return;
	printf("inserting unserved nodes .... \n");
	// else, node unserved, insert them infeasibly.
	int i;
	for (i = 0; i < dvrp_node_count; i++)
	{
		if (dvrp_node_set[i].isinserted == false)
		{
			insert_node_sdvrp(dvrp_node_set[i]);
			dvrp_node_set[i].isinserted = true;
		}
	}
}


void Solution::end_static_handler()
{
	dvrp_node_count = 0;
}

// static
// need a re-routing function, in this re-routing function we need to find out new allocation first 
void Solution::NA_handler_static(NodeArrival na_event)
{
	// collect nodes
	printf("Node Arrival handler, static version .... \n");
	collect_nodes();
	// add new arrival node into node set
	Client tmp_client;
	tmp_client.init(na_event);
	add_into_node_set(tmp_client);
	// rebuild solution
	rebuild();
	// re-route based on existing route and not-yet visited node set
	// i.e. route from current time
	// Later will test tabu search, now test construction first
	route_from_current();
	// re-insert unserved nodes
	insert_unserved_nodes();
	// end handler, clear the dvrp_node_set;
	end_static_handler();
}

void Solution::NC_handler_static(NodeCancellation nc_event)
{
	// make the cancellation and re-route
	change_supply((Event *)&nc_event);
	collect_nodes();
	rebuild();
	route_from_current();
	insert_unserved_nodes();
	end_static_handler();
}

void Solution::NV_handler_static(NegativeVariation nv_event)
{
	// make the negative variation and re-route
	// question: do we need to relax the demand? -- new allocation
	change_supply((Event *)&nv_event);
	collect_nodes();
	rebuild();
	route_from_current();
	insert_unserved_nodes();
	end_static_handler();
}

void Solution::PV_handler_static(PositiveVariation pv_event)
{
	// make the positive variation and re-route
	change_supply((Event *)&pv_event);
	collect_nodes();
	rebuild();
	route_from_current();
	insert_unserved_nodes();
	end_static_handler();
}

// Note: every node based event must be handled -- detected if any unhandled
//       there may be unhandled time based events, print them
void Solution::find_unhandled_events()
{
	int i;
	int tmp_count = 0;
	for (i = 0; i < event_count_node; i++)
	{
		if (pevent_node[i]->ishandled == false)
		{
			printf("unhandled node based event detected!\n");
			pevent_node[i]->show();
			tmp_count++;
			// system("pause");
		}
	}
	for (i = 0; i < event_count_time; i++)
	{
		if (((NodeArrival*)pevent_time[i])->successfully_handled == false)
		{
			printf("new arrival did not handled successfully!\n");
			pevent_time[i]->show();
		}

		if (pevent_time[i]->ishandled == false)
		{
			printf("event not handled:\n");
			tmp_count++;

			// bug detect
			if (pevent_time[i]->time < end_time)
			{
				printf("bug detected: unhandled time based event before finishing routing!\n");
				pevent_node[i]->show();
				system("pause");
			}
			else
			{
				printf("unhandled time based event after finishing routing!\n");
				pevent_time[i]->show();
			}
		}
	}
	if (tmp_count == 0)
	{
		printf("all events handled!\n");
	}
	else
	{
		printf("%d events have not handled\n");
	}
	return;
}