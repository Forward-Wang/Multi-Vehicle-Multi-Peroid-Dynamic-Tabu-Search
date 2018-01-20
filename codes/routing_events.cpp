#include "head.h"
#include "Routing.h"
#include "event.h"
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

extern Vehicle vehicle[VEHICLE_LIMIT];
extern Client client[CLIENT_LIMIT];
extern double costmatrix[CLIENT_LIMIT][CLIENT_LIMIT];
extern Solution bestsolution;

void Routing::read_events(int caseid)
{
	// DEBUG: need to test the corretness here

	printf("reading events .... \n");

	ifstream fd;
	string file_name = "EventData";
	char buff[16];
	char readbuff[512];
	char typebuff[16];
	int i, i_time, i_node;
	double tmptime;
	int tmpid;
	double tmpx;
	double tmpy;
	double tmp_perishable_supply;
	double tmp_longtime_supply;
	double tmp_perishable_decrease;
	double tmp_longtime_decrease;
	double tmp_perishable_increase;
	double tmp_longtime_increase;

	_itoa(caseid, buff, 10);
	file_name += string(buff);
	file_name += ".txt";
	fd.open(file_name);
	if (!fd)
	{
		printf("cannot open %s, exit!\n", file_name.c_str());
		system("pause");
		exit(1);
	}
	fd.getline(readbuff, sizeof(readbuff));
	sscanf_s(readbuff, "%d", &Event::totalevent);
	printf("%d events in total!\n", Event::totalevent);

	i_time = 0;
	i_node = 0;

	for (i = 0; i < Event::totalevent; i++)
	{
		fd.getline(readbuff, sizeof(readbuff));

		strncpy(typebuff, readbuff, 2);
		typebuff[2] = '\0';

		// printf("i = %d\n", i);
		// printf("readbuff: %s\n", readbuff);
		// printf("typebuff: %s\n", typebuff);
		
		if (strcmp(typebuff, "NA") == 0)
		{
			sscanf_s(&readbuff[3], "%lf %d %lf %lf %lf %lf", 
					 &tmptime, 
					 &tmpid, 
					 &tmpx, 
					 &tmpy, 
					 &tmp_perishable_supply, 
					 &tmp_longtime_supply);
			tmpid = assign_new_id();
			pevent[i] = new NodeArrival(tmpid, tmptime, tmpx, tmpy, tmp_perishable_supply, tmp_longtime_supply);
			pevent_time[i_time] = pevent[i];
			i_time++;
			// printf("A NA event!\n");
			// pevent[i]->show();
		}
		else if (strcmp(typebuff, "NV") == 0)
		{
			sscanf_s(&readbuff[3], "%lf %d %lf %lf",
				&tmptime,
				&tmpid,
				&tmp_perishable_decrease,
				&tmp_longtime_decrease);
			pevent[i] = new NegativeVariation(tmpid, tmptime, tmp_perishable_decrease, tmp_longtime_decrease);
			pevent_node[i_node] = pevent[i];
			i_node++;
			// printf("A NV event!\n");
			// pevent[i]->show();
		}
		else if (strcmp(typebuff, "NC") == 0)
		{
			sscanf_s(&readbuff[3], "%lf %d",
				&tmptime,
				&tmpid);
			pevent[i] = new NodeCancellation(tmpid, tmptime);
			pevent_node[i_node] = pevent[i];
			i_node++;
			//printf("A NC event!\n");
			//pevent[i]->show();
		}
		else if (strcmp(typebuff, "PV") == 0)
		{
			sscanf_s(&readbuff[3], "%lf %d %lf %lf",
				&tmptime,
				&tmpid,
				&tmp_perishable_increase,
				&tmp_longtime_increase);
			pevent[i] = new PositiveVariation(tmpid, tmptime, tmp_perishable_increase, tmp_longtime_increase);
			pevent_node[i_node] = pevent[i];
			i_node++;
			//printf("A PV event!\n");
			//pevent[i]->show();
		}
		else
		{
			printf("wrong EventData file! please check the input !\n");
			system("pause");
			exit(1);
		}
	}
	event_count_time = i_time;
	event_count_node = i_node;
}

int Routing::assign_new_id()
{
	new_id++;
	return new_id - 1;
}

// is this node corresponding to a particular event?
// if true, return the pointer to this event, else return false
Event * Routing::is_event_node(Client node)
{
	int i;
	for (i = 0; i < event_count_node; i++)
	{
		if (pevent_node[i]->node_id == node.id && pevent_node[i]->ishandled == false)
		{
			return pevent_node[i];
		}
	}
	return NULL;
}

// after handling a event, if a good solution becomes better than the current best solution, change them
// we need to do this because we choose the next upcoming location based event based on best solution, which
// means that the best solution has higher priority than good solutions
// good solution may be not applicable thus being abandoned, but best solution will always be kept
void Routing::check_good_solutions()
{
	printf("checking if better solution found .... \n");
	int i;
	int tmpi = -1;
	double tmp_best_cost = bestsolution.solutioncost;
	for (i = 0; i < good_solutions.count; i++)
	{
		tmpi = better_solution(tmpi, i);
	}
	// test: always let good solution interchanges with best solution
	// tmpi = 0;
	if (tmpi != -1)
	{
		printf("after this event, good solution %d becomes best solution!\n", tmpi);
		printf("previous: dvrp_feasibility = %d, waste = %lf, cost = %lf\n", 
			bestsolution.isfea_after_current, bestsolution.waste, bestsolution.solutioncost);
		printf("now:      dvrp_feasibility = %d, waste = %lf, cost = %lf\n",
			good_solutions.solutions[tmpi].isfea_after_current, 
			good_solutions.solutions[tmpi].waste, 
			good_solutions.solutions[tmpi].solutioncost);
		tmpsolution = bestsolution;
		bestsolution = good_solutions.solutions[tmpi];
		good_solutions.solutions[tmpi] = tmpsolution;
	}
	else
	{
		printf("no good solution become better solution!\n");
	}
	return;
}

// solution priority: 1. feasibility 2. waste 3. time cost
int Routing::better_solution(int i, int j)
{
	Solution *pi, *pj;
	if (i == -1)
	{
		pi = &bestsolution;
	}
	else
	{
		pi = &good_solutions.solutions[i];
	}
	pj = &good_solutions.solutions[j];
	// feasibility
	if (pi->isfea_after_current == true && pj->isfea_after_current == false)
	{
		return i;
	}
	else if (pi->isfea_after_current == false && pj->isfea_after_current == true)
	{
		return j;
	}
	// all not feasible, or all feasible
	else
	{
		if (pi->equity < pj->equity)
		{
			return i;
		}
		else if (pi->equity > pj->equity)
		{
			return j;
		}
		else
		{
			// waste
			if (pi->waste < pj->waste)
			{
				return i;
			}
			else if (pi->waste > pj->waste)
			{
				return j;
			}
			// same waste, check time cost
			else
			{
				if (pi->solutioncost <= pj->solutioncost)
				{
					return i;
				}
				else
				{
					return j;
				}
			}
		}
	}
}

// Note: timing system
// - a global <current_time> for all solutions
// - a local <start_time> for each vehicle, can be in the middle of a day
// - when we calculate the current location of a vehicle, we will calculate that based on current_time - start_time
// How to maintain the timing system?
// - suppose current_time = t0
// - based on t0, find out the next upcoming event
// - for all solutions:
//   - let all vehicles finish the nodes right after that event, and update current time
//     if that event is a location based event, let the vehicle finish that node
//   - when finishing nodes, put other events on the way into the event buffer
//     this buffer is organised as a queue, based on the time of these events
//     and will be handled sequentially later
// the whole routing system will go with the following pattern:
// iteration: -- find next upcoming event 
//            -- vehicles finish nodes right after that event
//               put events amoung the way into an event buffer
//               update current time 
//            -- handle next upcoming event 
//            -- handle buffered events 
// see Timing system.docx for more info 

// -- DONE
void Routing::route_events_main()
{
	printf("start routing events .... \n");

	int i;
	double next_current_time = 0;

	clock_t tmp_start_time = clock();
	clock_t tmp_end_time;

	// best solution push forward time, collect events on its way
	next_current_time = bestsolution.push_forward_time();

	// if next_current_time == -1, best solution routing finished
	while (next_current_time > 0)
	{
		bestsolution.batch_event_handling();

		for (i = 0; i < good_solutions.count; i++)
		{
			// let the good solutions follw next current time, and collect events on its way
			good_solutions.solutions[i].follow_next_current(next_current_time);
			good_solutions.solutions[i].batch_event_handling();
		}

		// interchange good solutions and best solution
		check_good_solutions();

		// best solution push forward time, collect events on its way
		next_current_time = bestsolution.push_forward_time();
	}

	// best solution ends event handling
	printf("best solution ends event handling\n");

	// push all good events to the end
	for (i = 0; i < good_solutions.count; i++)
	{
		// let the good solutions follw next current time, and collect events on its way
		good_solutions.solutions[i].follow_next_current(-1);
		good_solutions.solutions[i].batch_event_handling();
	}

	// interchange good solutions and best solution
	check_good_solutions();

	// detect if unhandled events for best solution
	bestsolution.find_unhandled_events();

	tmp_end_time = clock();
	if (REACTION_TYPE == DYNAMIC_HANDLER)
	{
		dynamic_time_cost = ((double)(tmp_end_time - tmp_start_time)) / CLOCKS_PER_SEC;
	}
	else
	{
		static_time_cost = ((double)(tmp_end_time - tmp_start_time)) / CLOCKS_PER_SEC;
	}

	// end event handling
	printf("routing events finished .... \n");
	bestsolution.dynamic_time_cost = dynamic_time_cost;
	bestsolution.static_time_cost = static_time_cost;
	bestsolution.show();
	bestsolution.write(fdoutx);
	return;
}

