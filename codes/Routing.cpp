#include "head.h"
#include "Routing.h"
#include "tabu.h"
#include <iostream>
#include <string>

using namespace std;

extern Vehicle vehicle[VEHICLE_LIMIT];
extern Client client[CLIENT_LIMIT];
extern Client depot;
extern double costmatrix[CLIENT_LIMIT][CLIENT_LIMIT];
extern Solution bestsolution;

// tabu parameter 
extern double lumbda;
extern double alpha;
extern double beta;
extern double gamma;
extern double mdfactor;

extern Routing routing;

void shift_operator(Solution &tpsolution, Shiftrecord srd);

// compare time sequence of two events
static int event_time_compare(const void *a, const void *b)
{
	Event *pa = (Event *)a;
	Event *pb = (Event *)b;

	if (pa->time < pb->time) return -1;
	else if (pa->time > pb->time) return 1;
	else return 0;
}

// ---- Initializer ----
void Routing::init()
{
	// what is the reaction when event happens?
	REACTION_TYPE = DYNAMIC_HANDLER;

	dynamic_time_cost = 0;
	static_time_cost = 0;

	// if first time to test this case, set this variable to be false
	// else, set it to be true to reproduce the same output
	EVENT_DATA_EXIST = false;

	// tabu parameter 
	lumbda = 0.02;
	alpha = 1;
	beta = 1;
	gamma = 1;
	mdfactor = 2;

	// new id to be assigned to arrival nodes
	new_id = CLIENT_LIMIT;

	// current time when doing routing
	current_time = EPSILON;
	EPSILON = 1e-4;
	TIME_LIMIT = 1000000;
	event_time_ptr = 0;

	// sort time-based events based on time
	qsort(pevent_time, event_count_time, sizeof(Event *), event_time_compare);

	// initialise best solution
	bestsolution.init();

	// DEBUG: check point here
	return;
}

// clear event pointers, and then delete all nodes 
void Routing::end_good_solutions()
{
	good_solutions.count = 0;
	vector <Solution>::iterator si;
	for (si = good_solutions.solutions.begin(); si != good_solutions.solutions.end(); si++)
	{
		(*si).clear_event_pointers();
	}
	good_solutions.solutions.clear();
}

void Routing::clear_event_pointers()
{
	// clear event pointer
	int i;
	for (i = 0; i < EVENT_LIMIT; i++)
	{
		/*if (pevent[i] != NULL)
		{
			delete pevent[i];
		}*/
		if (pevent_node[i] != NULL)
		{
			delete pevent_node[i];
		}
		if (pevent_time[i] != NULL)
		{
			delete pevent_time[i];
		}
		pevent[i] = NULL;
		pevent_node[i] = NULL;
		pevent_time[i] = NULL;
	}
}

// check the minimum timp gap between two nodes is larger than EPSILON
// for later use in our time system
// this function is based on the linked list structure of the routes, NEED TO MAKE SURE OF THIS
void Routing::check_min_time_gap()
{
	int i, j;
	double prev_time;
	double tmp_gap;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (bestsolution.vehiclelist[i].len <= 2) continue;
		prev_time = 0;
		for (j = 1; j < bestsolution.vehiclelist[i].len; j++)
		{
			tmp_gap = bestsolution.vehiclelist[i].route[j].cost - prev_time;
			if (tmp_gap < EPSILON) EPSILON = 0.1 * tmp_gap;
			prev_time = bestsolution.vehiclelist[i].route[j].cost;
		}
	}
	// debug point here!
	printf("setting minimum time gap EPSILON = %lf\n", EPSILON);
	return;
}

// ---- run routing ----
void Routing::run(int caseid)
{

	clock_t tmp_start_time = clock();
	clock_t tmp_end_time;

	init();
	file_input(caseid);
	build_init_solution();
	write("--------\n\nafter initial construction\n\n");

	perform_tabu_search();
	write("--------\n\nafter tabu search\n\n");

	perform_opt();
	write("--------\n\nafter opt operator\n\n");

	perfrom_waste_distribution();
	write("--------\n\nafter waste distribution\n\n");

	if (EVENT_DATA_EXIST == false)
	{
		printf("write to file and generate input for case generator .... \n");
		write_generator_input(caseid);
		printf("successfully write to file, start case generator .... \n");
		string generator_command = "python event_generator_170106.py -";
		char buff[10];
		_itoa(caseid, buff, 10);
		generator_command += string(buff);
		system(generator_command.c_str());
		// system("python test.py");
		printf("generator finished .... \n");
	}

	tmp_end_time = clock() - tmp_start_time;
	double time_static = ((double)(tmp_end_time - tmp_start_time)) / CLOCKS_PER_SEC;
	
	read_events(caseid);
	
	event_handling_init(time_static);

	// main function for routing
	route_events_main();
	
	write("--------\n\nafter routing events\n\n");
	bestsolution.write_info(fdoutx);
	end_case(caseid);
	
	return;
}

static void copy_event(Event **dst, Event *src)
{
	// copy this event to bestsolution's private event list
	// note the type conversion trick here
	if (src->type == NC)
	{
		NodeCancellation *tmp = new NodeCancellation();
		*tmp = *(NodeCancellation *) src;
		*dst = tmp;
	}
	else if (src->type == NA)
	{
		NodeArrival *tmp = new NodeArrival();
		*tmp = *(NodeArrival *) src;
		*dst = tmp;
	}
	else if (src->type == NV)
	{
		NegativeVariation *tmp = new NegativeVariation();
		*tmp = *(NegativeVariation *) src;
		*dst = tmp;
	}
	else if (src->type == PV)
	{
		PositiveVariation *tmp = new PositiveVariation();
		*tmp = *(PositiveVariation *) src;
		*dst = tmp;
	}
	else
	{
		// debug
		printf("wrong event type, please check again!\n");
		system("pause");
	}
}

// copy all events to solutions' private event list
// simply call event init function for all solutions
void Routing::event_handling_init(double time_static)
{
	printf("---- Initialising event handling .... \n");

	int i, j;

	// Initialize EPSILON
	check_min_time_gap();

	// initialize event handling
	bestsolution.event_handling_init();
	bestsolution.current_time = EPSILON;
	bestsolution.REACTION_TYPE = REACTION_TYPE;
	bestsolution.TIME_LIMIT = TIME_LIMIT;
	bestsolution.EPSILON = EPSILON;
	bestsolution.pfdoutx = &fdoutx;

	double avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery;
	double tmpwaste;
	bestsolution.get_fillrate(avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery);
	bestsolution.get_waste(tmpwaste);
	bestsolution.info_list[bestsolution.info_count].set_value(time_static, 
		bestsolution.solutioncost, avg_fillrate_delivery, min_fillrate_delivery, 
		max_fillrate_delivery, tmpwaste, -1, -1, -1);
	bestsolution.info_count += 1;

	for (i = 0; i < good_solutions.count; i++)
	{
		good_solutions.solutions[i].event_handling_init();
		good_solutions.solutions[i].REACTION_TYPE = REACTION_TYPE;
		good_solutions.solutions[i].current_time = EPSILON;
		good_solutions.solutions[i].TIME_LIMIT = TIME_LIMIT;
		good_solutions.solutions[i].EPSILON = EPSILON;
		good_solutions.solutions[i].pfdoutx = &fdoutx;

		double avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery;
		double tmpwaste;
		good_solutions.solutions[i].get_fillrate(avg_fillrate_delivery, min_fillrate_delivery, max_fillrate_delivery);
		good_solutions.solutions[i].get_waste(tmpwaste);
		good_solutions.solutions[i].info_list[good_solutions.solutions[i].info_count].set_value(
			time_static, good_solutions.solutions[i].solutioncost, 
			avg_fillrate_delivery, min_fillrate_delivery,
			max_fillrate_delivery, tmpwaste, -1, -1, -1);
		good_solutions.solutions[i].info_count += 1;
	}

	// copy time based events
	for (i = 0; i < event_count_time; i++)
	{
		// copy this event to bestsolution's private event list
		copy_event(&bestsolution.pevent_time[i], pevent_time[i]);
		if (bestsolution.pevent_time[i] == NULL)
		{
			printf("did not successfully copy event!\n");
		}
		// copy to good solutions
		for (j = 0; j < good_solutions.count; j++)
		{
			copy_event(&good_solutions.solutions[j].pevent_time[i], pevent_time[i]);
		}
	}
	// copy count
	bestsolution.event_count_time = event_count_time;
	for (j = 0; j < good_solutions.count; j++)
	{
		good_solutions.solutions[j].event_count_time = event_count_time;
	}

	// copy location based events
	for (i = 0; i < event_count_node; i++)
	{
		copy_event(&bestsolution.pevent_node[i], pevent_node[i]);
		// copy to good solutions
		for (j = 0; j < good_solutions.count; j++)
		{
			copy_event(&good_solutions.solutions[j].pevent_node[i], pevent_node[i]);
		}
	}
	// copy count
	bestsolution.event_count_node = event_count_node;
	for (j = 0; j < good_solutions.count; j++)
	{
		good_solutions.solutions[j].event_count_node = event_count_node;
	}

	printf("---- event handling initialization finished .... \n");
}

// ---- clear all previous case data, read all the input files ----
void Routing::file_input(int caseindex)
{
	// ---- definition ----
	// File names
	string sclientfile = "ClientData";
	string svehiclefile = "VehicleData";
	string scostmatrixfile = "CostMatrix";
	string seventfile = "EventData";
	string soutfile = "outx";

	string clientfile;
	string vehiclefile;
	string costmatrixfile;
	string eventfile;
	string outfile;

	char filebuffer[10];
	char readbuffer[20000];

	int bufferindex;
	int i;
	int j;

	// File discripters
	ifstream fdclient;
	ifstream fdmatrix;
	ifstream fdvehicle;
	ifstream fdevent;

	// Processing filename string
	_itoa_s(caseindex, filebuffer, 10);
	costmatrixfile = scostmatrixfile;
	clientfile = sclientfile;
	vehiclefile = svehiclefile;
	eventfile = seventfile;
	outfile = soutfile;

	costmatrixfile.append(filebuffer);
	clientfile.append(filebuffer);
	vehiclefile.append(filebuffer);
	eventfile.append(filebuffer);
	outfile.append(filebuffer);

	costmatrixfile.append(".txt");
	clientfile.append(".txt");
	vehiclefile.append(".txt");
	eventfile.append(".txt");
	outfile.append(".txt");


	// ---- Open file streams ---- 
	// costmatrix
	printf("Opening file %s\n", costmatrixfile.c_str());
	fdmatrix.open(costmatrixfile);
    if(!fdmatrix)
    {
		printf("Fail to open %s, please make sure your working directory contains this file!\n", costmatrixfile.c_str());
		system("pause");
        exit(1);
    }

	// client
	printf("Opening file %s\n", clientfile.c_str());
	fdclient.open(clientfile);
    if(!fdclient)
    {
		printf("Fail to open %s, please make sure your working directory contains this file!\n", clientfile.c_str());
		system("pause");
        exit(1);
    }

	// vehicle
	printf("Opening file %s\n", vehiclefile.c_str());
	fdvehicle.open(vehiclefile);
    if(!fdvehicle)
    {
        printf("Fail to open %s, please make sure your working directory contains this file!\n", vehiclefile.c_str());
		system("pause");
        exit(1);
    }

	// event
	/*
	printf("Opening file %s\n", eventfile.c_str());
	fdevent.open(eventfile);
    if(!fdevent)
    {
        printf("Fail to open %s, please make sure your working directory contains this file!\n", eventfile.c_str());
		system("pause");
        exit(1);
    }*/

	// output
	printf("Opening file %s\n", outfile.c_str());
	fdoutx.open(outfile);
  if(!fdoutx)
  {
	printf("Fail to open %s, please make sure your working directory contains this file!\n", soutfile.c_str());
	system("pause");
      exit(1);
  }

	// clean the file content of last iteration and then read the file for this iteration


	// ---- read cost matrix ----
	// 
	printf("reading cost matrix\n");
  fdmatrix.getline(readbuffer, sizeof(readbuffer));
  sscanf_s(readbuffer, "%d", &Client::totalnode);
  printf("Total number of node = %d\n", Client::totalnode);

	// new_id = Client::totalnode + 1000;

	// check the total node limit
	if(Client::totalnode > CLIENT_LIMIT)
  {
	printf("Too many nodes! Currently our limit is %d, contact the developer to get more nodes supported.", CLIENT_LIMIT);
	system("pause");
      exit(1);
  }

	// read all matrix data
  for(i = 0; i < Client::totalnode; i++)
  {
    fdmatrix.getline(readbuffer, sizeof(readbuffer));
    bufferindex = 0;
		// printf("buffer: %s\n", readbuffer);
    for(j = 0; j < Client::totalnode; j++)
    {
      sscanf_s(&readbuffer[bufferindex], "%lf", &costmatrix[i][j]);
			// printf("matrix %d %d: %lf \n", i, j, costmatrix[i][j]);
      if(j < Client::totalnode - 1)
      {
        while((readbuffer[bufferindex] >= '0' && readbuffer[bufferindex] <= '9') || readbuffer[bufferindex] == '.') bufferindex++;
        while(!((readbuffer[bufferindex] >= '0' && readbuffer[bufferindex] <= '9') || readbuffer[bufferindex] == '.')) bufferindex++;
      }
    }
  }
  fdmatrix.close();


	// ---- read client data ----
	// tell the difference between two kinds of nodes 
	printf("reading client data ... \n");
	fdclient.getline(readbuffer, sizeof(readbuffer));
	for(i = 0; i < Client::totalnode; i++)
   {
		fdclient.getline(readbuffer, sizeof(readbuffer));
		sscanf_s(readbuffer, "%d %d", &client[i].id, &client[i].type);
		if(client[i].type == PICK)
		{
			sscanf_s(readbuffer, "%d %d %lf %lf %lf %lf %d %lf %lf %lf", &client[i].id, &client[i].type, &client[i].xcoordinate, &client[i].ycoordinate,
				&client[i].long_supply, &client[i].prsb_supply, &client[i].frequency, &client[i].opentime, &client[i].closetime, &client[i].servicetime);
		}
		else
		{
			sscanf_s(readbuffer, "%d %d %lf %lf %lf %d %lf %lf %lf", &client[i].id, &client[i].type, &client[i].xcoordinate, &client[i].ycoordinate,
				&client[i].demand, &client[i].frequency, &client[i].opentime, &client[i].closetime, &client[i].servicetime);
		}
        client[i].count_distance();
		// client[i].show();
    }
	depot = client[0];
	get_reduced_demand();
    fdclient.close();

	// ---- read vehicle data ----
	printf("reading vehicle data ... \n");
    fdvehicle.getline(readbuffer, sizeof(readbuffer));
    sscanf_s(readbuffer, "%d", &Vehicle::totalvehicle);
    printf("\nTotal vehicle: %d\n", Vehicle::totalvehicle);

	// check vehicle number limitation
	if(Vehicle::totalvehicle > VEHICLE_LIMIT)
    {
        printf("Too many vehicles! Please modify the codes to support more vehicles!");
		system("pause");
        exit(1);
    }

	// read all data
    for(i = 1; i <= Vehicle::totalvehicle; i++)
    {
        fdvehicle.getline(readbuffer, sizeof(readbuffer));
        sscanf_s(readbuffer, "%d %lf %lf %lf", &vehicle[i].id, &vehicle[i].capacity, &vehicle[i].opentime, &vehicle[i].closetime);
		vehicle[i].relaxted_closetime = vehicle[i].closetime * TIME_RELAXATION;
    }
    fdvehicle.close();

	// ---- read event data ----

	/*
	printf("reading event data ... \n");
	fdevent.getline(readbuffer, sizeof(readbuffer));
    sscanf_s(readbuffer, "%d", &Event::totalevent);
    printf("\nTotal event: %d\n", Event::totalevent);

	// check vehicle number limitation
	if(Event::totalevent > EVENT_LIMIT)
    {
        printf("Too many events! Please modify the .h file to support more events!");
		system("pause");
        exit(1);
    }

	// start to read
	j = 0;
	k = 0;
	l = 0;
	m = 0;
	int eventi = 0;
	total_time_based_event = 0;
	for(i = 0; i < Event::totalevent; i++)
    {
        fdevent.getline(readbuffer, sizeof(readbuffer));
		// printf("have read: %s\n", readbuffer);
		sscanf(readbuffer, "%s %lf", eventbuffer, &tmpeventtime);
		// printf("event:%s, time:%lf", eventbuffer, tmpeventtime);
		// new arrival
		if(strcmp(eventbuffer, "NA") == 0)
		{
			sscanf(readbuffer, "%s %lf %lf %lf %lf %lf", eventbuffer, &newarrival[j].time, &newarrival[j].x, &newarrival[j].y,
				&newarrival[j].lsupply, &newarrival[j].psupply);

			// newarrival[j].type = string(eventbuffer);
			events[eventi].type = NA;
			events[eventi].time = newarrival[j].time;
			totalNA++;
			// newarrival[j].show();
			eventi++;
			j++;
		}
		// node disappear
		else if(strcmp(eventbuffer, "ND") == 0)
		{
			sscanf(readbuffer, "%s %lf %d", eventbuffer, &nodedisappear[k].time, &nodedisappear[k].id);
			// nodedisappear[k].type = string(eventbuffer);
			events[eventi].type = ND;
			events[eventi].time = nodedisappear[k].time;
			totalND++;
			// disappearnode[k].show();
			eventi++;
			k++;
		}
		// change of demand
		// this is not a event 
		else if(strcmp(eventbuffer, "DC") == 0)
		{
			sscanf(readbuffer, "%s %lf %d %lf", eventbuffer, &demandchange[l].time, &demandchange[l].id, &demandchange[l].newdemand);
			// demandchange[l].type = string(eventbuffer);
			l++;
		}
		// change of supply
		else if(strcmp(eventbuffer, "SC") == 0)
		{
			sscanf(readbuffer, "%s %lf %d %lf %lf", eventbuffer, &supplychange[m].time, &supplychange[m].id, 
				&supplychange[m].newlsupply, &supplychange[m].newpsupply);
			// supplychange[m].type = string(eventbuffer);
			// events[i].type = SC;
			// events[i].time = supplychange[m].time;
			totalSC++;
			// supplychange[m].show();
			m++;
		}
		else
		{
			printf("Error: cannot parse event!\n");
			system("pause");
			exit(1);
		}
    }
	total_time_based_event = eventi;
    fdevent.close();
	*/
	return;
}

// ---- get reduced demand of clients ----
void Routing::get_reduced_demand()
{
	double totaldemand = 0;
	double totalsupply = 0;
	double ratio;
	int i;
	for (i = 0; i < Client::totalnode; i++)
	{
		if (client[i].type == PICK)
		{
			totalsupply += client[i].long_supply + client[i].prsb_supply;
		}
		else
		{
			totaldemand += client[i].demand;
		}
	}
	ratio = totalsupply / totaldemand;
	printf("Total pickup: %lf, total delivery: %lf\n", totalsupply, totaldemand);
	printf("the ratio: supply / demand %lf", ratio);
	for (i = 0; i < Client::totalnode; i++)
	{
		if (client[i].type == DELV)
		{
			client[i].init_demand = client[i].demand;
			client[i].demand = client[i].demand * ratio;
			client[i].reduced_demand = client[i].init_demand * ratio * RELAX_RATIO;
		}
	}
	return;
}

// ---- build initial solution ----
void Routing::build_init_solution()
{
	// if initial solution is built successfully, continue
	if(build_solution_direct())
	{
		printf("successfully build initial solution directly!\n");
	}

	// else, use clustering 
	else
	{
		printf("still node unserved, use clustering ....\n");
		// print unserved nodes 
		if(build_solution_clustering_init())
		{
			printf("successfully build initial solution using clustering!\n");
		}
		else
		{
			printf("still unserved nodes after clustering, pass it!\n");
			// sleep for one second
		}
	}

	// perform rearrange operator
	bestsolution.perform_rearrange();
	// perform swap operator
	bestsolution.perform_swap();
	return;
}


// ---- use tabu search to perform routing ----
void Routing::perform_tabu_search()
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
	double neighbormf;
	double tabucost;
	int feasibilityret_tabu;
	Shiftrecord shiftobj;

	printf("\n\n\nStart tabu search...\n");
	// printf("Initializing...\n");

	// printf("Initial best solution:\n");
	bestsolution.eval();
	// bestsolution.show();
	bestsolution.set_sttable();
	// bestsolution.show();

	currentsolution = bestsolution;
	// currentsolution.show();

	// start the search algorithm, stop criteria check
	tabu_para_reset();
	tahistory.reset();
	for (i = 0; i < STOPCRITERIA; i++)
	{
	stageii:
		// Please note that every time control reaches here, current solution should be prepared
		// Delete the history of tmpsolution, prevent memory leak
		// Please make sure every time you want to copy a solution to another, you update the destination solution first.
		// We will optimize this to save the update labour work in the future
		// tmpsolution.update();

		// Copy the current solution to the tmp solution
		// printf("i = %d\n", i);
		// currentsolution.show();;
		// solution_copy(currentsolution, tmpsolution);
		tmpsolution = currentsolution;
		// copy_sttable(currentsolution, tmpsolution);

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
		shiftobj = currentsolution.choose_shift_st(tahistory);
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

		// printf("Before shift\n");
		// tmpsolution.show();
		// printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
		// shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.nodepos, shiftobj.vehicletoid, shiftobj.position);

		shift_operator(tmpsolution, shiftobj);
		//printf("Move node %d from vehicle%d to vehicle%d.\n", shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.vehicletoid);

		// use eval() to evaluate the value: evalfunc, mevalfunc and solutioncost
		// note that when a solution is feasible, we will have: evalfunc = solutioncost
		tmpsolution.eval();
		// printf("After eval\n");
		// tmpsolution.show();

		// Check the feasibility.
		// This function will check the feasibility of all the vehicles
		// return 0: feasible; 1: infeasible
		feasibilityret = tmpsolution.feasibility_check();
		if (feasibilityret == 0)
		{
			total_fea++;
		}
		else
		{
			total_infea++;
		}

		// If a better solution
		if ((feasibilityret == 0) && (tmpsolution.evalfunc < bestsolution.evalfunc))
		{

			printf("\n\n Find a better solution!i = %d\n", i);
			printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
				shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.nodepos, shiftobj.vehicletoid, shiftobj.position);

			printf("best solution: Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);
			printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", tmpsolution.solutioncost, tmpsolution.evalfunc);
			// printf("Before:\n");
			// bestsolution.show();
			// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);


			// Set the linked list to array
			tmpsolution.vehiclelist[shiftobj.vehiclefromid].linkedlist2array();
			tmpsolution.vehiclelist[shiftobj.vehicletoid].linkedlist2array();
			// tmpsolution.show();
			// printf("total vehicle %d\n", Vehicle::totalvehicle);

			// Update the operation history
			// bestsolution.update();
			// tmpsolution.update();

			// Set the best solution
			// printf("Find a better solution! Update to best solution!\n");
			// solution_copy(tmpsolution, bestsolution);

			// keep good solutions
			if (good_solutions.count < good_solutions.limit)
			{
				good_solutions.solutions.push_back(bestsolution);
				good_solutions.count++;
			}
			else
			{
				good_solutions.solutions.erase(good_solutions.solutions.begin());
				good_solutions.solutions.push_back(bestsolution);
			}

			bestsolution = tmpsolution;


			// printf("After:\n");
			// bestsolution.show();
			// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);


			// Reset tabu history
			tahistory.reset();

			// Reset sequential tabu table
			bestsolution.set_sttable();

			// Prepare the current solution
			// currentsolution.update();
			// solution_copy(bestsolution, currentsolution);
			currentsolution = bestsolution;
			tabu_para_reset();
		}
		else
		{
			// If a tabu solution, then begin/continue tabu search.
			// In our algorithm, we will treat the start of a new tabu search and continue an existing tabu search as the same.
			// Just the level of depth of tabu search is different
			if (feasibilityret == 1 && tmpsolution.evalfunc < currentsolution.evalfunc)
			{
				printf("\n\nFind a tabu solution!i = %d\n", i);
				printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
					shiftobj.nodeid, shiftobj.vehiclefromid, shiftobj.nodepos, shiftobj.vehicletoid, shiftobj.position);
				printf("current solution: Totalcost = %.lf, evalfunc = %.lf\n", currentsolution.solutioncost, currentsolution.evalfunc);
				printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", tmpsolution.solutioncost, tmpsolution.evalfunc);
				printf("Tabu depth: %d\n", tabu_cnt);
				// set the current solution
				total_tabu++;
				// currentsolution.update();
				// solution_copy(tmpsolution, currentsolution);
				currentsolution = tmpsolution;

				// Start/continue tabu search
				// If tabu_cnt == 0, then we start a new tabu search. If tabu_cnt > 1, means we are continuing an axisting rabu search


				// If the tabu search depth is 0, means we just find a tabu solution from best solution.
				// We will remember the sequantial tabu table in case we reach the bottom of tabu search, cannot find a inspired solution, roll back to bestsolution
				if (tabu_cnt == 0)
				{
					copy_sttable(currentsolution, bestsolution);
				}
				tabu_cnt++;

				// Tabu search reaches to the depth limit
				if (tabu_cnt == TABUTENURE)
				{
					// Roll back to last best solution
					// printf("Tabu limit exceeded, roll back to former best solution!\n\n\n");
					// currentsolution.update();
					// solution_copy(bestsolution, currentsolution);
					currentsolution = bestsolution;
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
					if (currentsolution.vehiclelist[j].len <= 2) continue;
					for (k = 1; k < Vehicle::totalvehicle; k++)
					{
						if (j == k || currentsolution.vehiclelist[k].len <= 2) continue;
						// for every node v in route of vehicle j
						for (v = 1; v < currentsolution.vehiclelist[j].len - 1; v++)
						{
							// and every possible position w in vehicle k
							// note this iteration is based on array structure
							for (w = 0; w < currentsolution.vehiclelist[k].len - 1; w++)
							{
								// if not in history record
								Shiftrecord tmpsrt;
								tmpsrt.vehiclefromid = j;
								tmpsrt.vehicletoid = k;
								tmpsrt.nodepos = v;
								tmpsrt.nodeid = currentsolution.vehiclelist[j].route[v].id;
								tmpsrt.position = w;
								// printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
								// tmpsrt.nodeid, tmpsrt.vehiclefromid, tmpsrt.nodepos, tmpsrt.vehicletoid, tmpsrt.position);
								if (tahistory.check_record(tmpsrt) == false)
								{
									// tmpsolution.update();
									// solution_copy(currentsolution, tmpsolution);
									tmpsolution = currentsolution;
									shift_operator(tmpsolution, tmpsrt);
									tmpsolution.eval();
									feasibilityret_tabu = tmpsolution.feasibility_check();

									// if aspired, i.e. feasible and cost reduced
									if ((feasibilityret_tabu == 0) && (tmpsolution.solutioncost < bestsolution.solutioncost))
									{
										printf("\n\nFind an aspired solution from neighbours. i = %d\n", i);
										printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
											tmpsrt.nodeid, tmpsrt.vehiclefromid, tmpsrt.nodepos, tmpsrt.vehicletoid, tmpsrt.position);
										printf("best solution: Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);
										printf("tmpsolution: Totalcost = %.lf, evalfunc = %.lf\n", tmpsolution.solutioncost, tmpsolution.evalfunc);										
										// printf("Before:\n");
										// bestsolution.show();
										// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);

										// Set the linked list to array
										tmpsolution.vehiclelist[tmpsrt.vehiclefromid].linkedlist2array();
										tmpsolution.vehiclelist[tmpsrt.vehicletoid].linkedlist2array();

										// Set the operation history
										// bestsolution.update();
										// tmpsolution.update();

										// Copy to best solution
										// solution_copy(tmpsolution, bestsolution);
										bestsolution = tmpsolution;
										bestsolution.set_sttable();

										// printf("After:\n");
										// bestsolution.show();
										// printf("Totalcost = %.lf, evalfunc = %.lf\n", bestsolution.solutioncost, bestsolution.evalfunc);

										// currentsolution.update();

										// Set the currentsolution
										// solution_copy(bestsolution, currentsolution);
										currentsolution = bestsolution;

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
										if (tmpsolution.mevalfunc < neighbormf)
										{
											// record the new better neighbor
											// printf("A better neighbour! mf: %f -> %f, neighborfrom %d\n", tmpsolution.mevalfunc, neighbormf, j);
											neighborfrom = j;
											neighborto = k;
											neighbornode = v;
											neighborpos = w;
											neighbormf = tmpsolution.mevalfunc;
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
					currentsolution = bestsolution;
					tabu_cnt = 0;

					// Clean the tabu history
					tahistory.reset();

					// Go back to the beginning of iteration
					tabu_para_reset();
					currentsolution.choose_shift_st(tahistory);
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
				tmpsrd.nodeid = currentsolution.vehiclelist[neighborfrom].route[neighbornode].id;
				printf("shift node %d from vehicle %d, position %d to vehicle %d, position %d\n",
					tmpsrd.nodeid, tmpsrd.vehiclefromid, tmpsrd.nodepos, tmpsrd.vehicletoid, tmpsrd.position);
				shift_operator(currentsolution, tmpsrd);
				// Evaluate the solution
				currentsolution.eval();

				// Update the operation history
				// currentsolution.update();

				// Reset the sttable
				currentsolution.set_sttable();
				//currentsolution.show();


				// update alpha, beta, gamma
				feasibilityret = currentsolution.feasibility_check();
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

	printf("tabu finished. cost: %lf\n", bestsolution.solutioncost);
	printf("After tabusearch, our final best solution is: \n");
	bestsolution.show();
	tabucost = bestsolution.solutioncost;

	printf("total tabu circle:%d\ttotal fea:%d\ttotal infea:%d\ttotal tabu:%d\n", tabu_total_cnt, total_fea, total_infea, total_tabu);
}

// ---- copy sequantial tabu table ----
// for tabu search
void copy_sttable(Solution &sour, Solution &dest)
{
	int i;
	for (i = 0; i < 100000; i++)
	{
		dest.sttable[i] = sour.sttable[i];
	}
	dest.stp = sour.stp;
	dest.possibilities = sour.possibilities;
	return;
}

Client get_node_from_event(int node_id)
{
	int i;
	int pid = -1;
	Client tmpclient;
	for (i = 0; i < routing.event_count_time; i++)
	{
		if (routing.pevent_time[i]->node_id == node_id)
		{
			pid = i;
			break;
		}
	}
	if (pid == -1)
	{
		printf("bug detected: cannot find event node!\n");
		system("pause");
	}
	tmpclient.init(*(NodeArrival *)routing.pevent_time[pid]);
	return tmpclient;
}

// ---- shift operator ----
void shift_operator(Solution &tpsolution, Shiftrecord srd)
{
	int nid = tpsolution.vehiclelist[srd.vehiclefromid].route[srd.nodepos].id;
	if (tpsolution.vehiclelist[srd.vehiclefromid].len == 2)
	{
		printf("shifting a node from empty route!\n");
		while (1);
	}
	if (nid != srd.nodeid)
	{
		printf("bad shift detected!\n");
		while (1);
	}
	// Insert the node, keep the linked list structure -- the linked list structure is wrong here
	tpsolution.vehiclelist[srd.vehicletoid].insert_i(nid, srd.position);
	// Maintain array structure
	tpsolution.vehiclelist[srd.vehicletoid].linkedlist2array();
	//tmpsolution.vehiclelist[srd.vehicletoid].vehicleroute.show();
	// Update the route, calculate all the violation
	// printf("Insert finished!\n");
	tpsolution.vehiclelist[srd.vehicletoid].update();
	//tmpsolution.vehiclelist[srd.vehicletoid].vehicleroute.show();
	// printf("Insert - update\n");
	// Kickout the node, keep the linked list structure
	tpsolution.vehiclelist[srd.vehiclefromid].kickout(srd.nodepos);
	// Maintain array structure
	tpsolution.vehiclelist[srd.vehiclefromid].linkedlist2array();
	// Update the route, calculate all the violation
	//tmpsolution.vehiclelist[srd.vehiclefromid].vehicleroute.show();
	tpsolution.vehiclelist[srd.vehiclefromid].update();
	//tmpsolution.vehiclelist[srd.vehiclefromid].vehicleroute.show();
	// Update the route structure
	// Update the number of vehicle in use
	// if(tpsolution.vehiclelist[srd.vehiclefromid].vehicleroute.len == 2) tpsolution.vehicleinuse--;
}

// ---- shift operator ----
void shift_operator_sdvrp(Solution &tpsolution, Shiftrecord srd)
{
	int nid = tpsolution.vehiclelist[srd.vehiclefromid].route[srd.nodepos].id;
	if (tpsolution.vehiclelist[srd.vehiclefromid].len == 2)
	{
		printf("shifting a node from empty route!\n");
		while (1);
	}
	if (nid != srd.nodeid)
	{
		printf("bad shift detected!\n");
		while (1);
	}
	// Insert the node, keep the linked list structure -- the linked list structure is wrong here
	tpsolution.vehiclelist[srd.vehicletoid].insert_i(nid, srd.position);
	// Maintain array structure
	tpsolution.vehiclelist[srd.vehicletoid].linkedlist2array();
	//tmpsolution.vehiclelist[srd.vehicletoid].vehicleroute.show();
	// Update the route, calculate all the violation
	// printf("Insert finished!\n");
	tpsolution.vehiclelist[srd.vehicletoid].update_sdvrp();
	//tmpsolution.vehiclelist[srd.vehicletoid].vehicleroute.show();
	// printf("Insert - update\n");
	// Kickout the node, keep the linked list structure
	tpsolution.vehiclelist[srd.vehiclefromid].kickout(srd.nodepos);
	// Maintain array structure
	tpsolution.vehiclelist[srd.vehiclefromid].linkedlist2array();
	// Update the route, calculate all the violation
	//tmpsolution.vehiclelist[srd.vehiclefromid].vehicleroute.show();
	tpsolution.vehiclelist[srd.vehiclefromid].update_sdvrp();
	//tmpsolution.vehiclelist[srd.vehiclefromid].vehicleroute.show();
	// Update the route structure
	// Update the number of vehicle in use
	// if(tpsolution.vehiclelist[srd.vehiclefromid].vehicleroute.len == 2) tpsolution.vehicleinuse--;
}

// ---- 2-opt operators on each vehicle ----
void Routing::perform_opt()
{
	printf("\n\n\nperforming opt operator...\n");
	int i = 0, j = 0, k = 0;
	Vehicle tmpvehicle;
	int tmpoptj = -1, tmpoptk = -1;
	int feasi_check_ret;
	double tmpcost;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		tmpoptj = -1, tmpoptk = -1;
		if (bestsolution.vehiclelist[i].len < 5) continue;
		tmpcost = bestsolution.vehiclelist[i].totalcost;
		for (j = 1; j < bestsolution.vehiclelist[i].len - 2; j++)
		{
			for (k = 1; k < bestsolution.vehiclelist[i].len - 1; k++)
			{
				if (j == k || j + 1 == k || k + 1 == j) continue;
				tmpvehicle = bestsolution.vehiclelist[i];
				tmpvehicle.opt_operator(j, k);
				feasi_check_ret = tmpvehicle.feasibility_check();
				if (tmpvehicle.totalcost < tmpcost && feasi_check_ret == 0)
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
		if (tmpoptj != -1)
		{
			printf("vehicle %d, choose the best opt: %d & %d. cost %lf -> %lf\n",
				i, tmpoptj, tmpoptk, bestsolution.vehiclelist[i].totalcost, tmpcost);
			bestsolution.vehiclelist[i].opt_operator(tmpoptj, tmpoptk);
			// bestsolution.vehiclelist[i].show();	
		}
		else
		{
			printf("vehicle %d: No better opt found!\n", i);
		}
	}
	printf("opt finished, update to bestsolution ...\n");
	bestsolution.show();
}

// ---- waste distribution on each vehicle ----
void Routing::perfrom_waste_distribution()
{
	int i;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (bestsolution.vehiclelist[i].len > 2)
		{
			bestsolution.vehiclelist[i].update();
			if (bestsolution.vehiclelist[i].distribute_waste())
			{
				printf("vehicle %d, distribute waste\n", i);
			}
			else
			{
				printf("vehicle %d, cannot distribute waste\n", i);
			}
		}
	}
	bestsolution.show();
}

// ---- end case ----
// write file, reclaim resources
void Routing::end_case(int caseid)
{
	bestsolution.clear_event_pointers();
	bestsolution.info_count = 0;
	// clear event pointers for all good solutions, and then delete good solutions
	end_good_solutions();
	// clear event pointers of this routing class
	clear_event_pointers();
	fdoutx.close();
	return;
}

// ---- directly build initial solution ---- 
// -- use the initial data to build and store in bestsolution
// -- will not use balance-ratio method anymore
// -- if success, return true
// -- if not success, store the unserved list and return false
// actually, this function is a little obsolete
bool Routing::build_solution_direct()
{
	double constructioncost = 0;
	int i, j, k, l;
	int jumpcnt;

	int tmpposition;
	double tmpmincost;
	double tmpcost;
	bool terminateflag;
	int feasibilityret;

	// Initialize the client
	for (i = 0; i < Client::totalnode; i++)
	{
		client[i].isinserted = false;
	}
	client[0].isinserted = true;

	// sort the clients according to its location
	qsort(client, Client::totalnode, sizeof(Client), clientcmp);

	// for every vacant vehicle
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		// Copy vehicle meta-data to tmpsolution
		tmpsolution.vehiclelist[i] = vehicle[i];

		// Route initialization -- do we need it here?
		// printf("Vehicle %d, initializing...\n", i);
		tmpsolution.vehiclelist[i].init(client);

		jumpcnt = 0;

		// try every node that has not been inserted
		for (j = 1; j < Client::totalnode; j++)
		{
			// to insert it in every possible route location in vehicle[i]'s route
			// terminal condition: 1> no avaiable node 2> vehicle time limit use up
			if (client[j].isinserted == true) // this is the flag to show whether this node is inserted
			{
				// pass the node that has already been inserted
				continue;
			}

			// initialize insert attempt
			tmpposition = -1;
			// the maxmum of the data type float 
			tmpmincost = numeric_limits<double>::max();
			// use this flag to detect termination
			terminateflag = false;

			k = tmpsolution.vehiclelist[i].head;

			// l runs through every possible position in the route of a vehicle
			for (l = 0; l < tmpsolution.vehiclelist[i].len - 1; l++)
			{
				// printf("Trying node %d at vehicle %d, position %d\n", client[j].id, i, l);
				feasibilityret = tmpsolution.vehiclelist[i].feasibility_check(j, k); // -- ok
				// here are the return value of feasibility_check function:
				// 0: feasible; 1: violate feasibility criteria 1,2,3; -1: violate feasibility criteria4
				// printf("trying insertion...\n");

				if (feasibilityret == 0)
				{
					// check if the cost will be reduced
					tmpcost = tmpsolution.vehiclelist[i].cost_calculate(j, k);
					if (tmpcost < tmpmincost)
					{
						// make a temporary record
						tmpmincost = tmpcost;
						tmpposition = k;
					}
					k = tmpsolution.vehiclelist[i].route[k].next;
				}

				else if (feasibilityret != -1) // violate feasibility criteria 1,2,3
				{
					// try next position
					k = tmpsolution.vehiclelist[i].route[k].next;
				}

				else // feasibilityret == -1, violate feasibility criteria4
				{
					// terminate
					printf("vehicle %d terninated!\n", i);
					terminateflag = true;
					// terminate detection codes
					// printf("terminate detected!\n");
					vehicle[i].show();
					// printf("node and position failed to insert: %d, %d", j, k);
					break;
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
				tmpsolution.vehiclelist[i].insert(j, tmpposition);
				client[j].isinserted = true;

				// print intermediate state
				// printf("a new insert!\n");
				// tmpsolution.vehiclelist[i].show();
				// tmpsolution.show();
				// detect insertion jump
				if (jumpcnt > 0)
				{
					j = 0;
					jumpcnt = 0;
				}
			}
		}

		// try out all node or terminated by feasibility criteria4
		printf("final route of vehicle %d:\n", i);
		tmpsolution.vehiclelist[i].show();
		constructioncost += tmpsolution.vehiclelist[i].totalcost;
	}

	tmpsolution.unservedclient = 0;
	printf("Detecting unserved clients...\n");
	
	for (i = 0; i < Client::totalnode; i++)
	{
		if (client[i].isinserted == false)
		{
			printf("client %d did not served!\n", client[i].id);
			tmpsolution.unservedlist[tmpsolution.unservedclient] = client[i].id;
			tmpsolution.unservedclient++;
		}
	}

	tmpsolution.solutioncost = 0;
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		if (tmpsolution.vehiclelist[i].len > 2) tmpsolution.solutioncost += tmpsolution.vehiclelist[i].totalcost;
	}

	bestsolution = tmpsolution;

	// process the solution, to array
	for (i = 1; i < Vehicle::totalvehicle; i++)
	{
		// printf("vehicle %d, len %d\n", i, bestsolution.vehiclelist[i].vehicleroute.len);
		if (bestsolution.vehiclelist[i].len > 2)
		{
			bestsolution.vehiclelist[i].linkedlist2array(); // -- ok
			// bestsolution.vehiclelist[i].vehicleroute.show();
		}
	}

	// then evaluate
	bestsolution.eval();

	printf("%d client left!\n", bestsolution.unservedclient);
	if (bestsolution.unservedclient > 0)
	{
		printf("\n\nHere are all left out clients:\n");
		for (i = 0; i < bestsolution.unservedclient; i++)
		{
			printf("node: %d  \n", bestsolution.unservedlist[i]);
		}
		return false;
	}

	// route construction finished
	printf("Route construction finished, store as bestsolution.\n");
	// bestsolution.show();
	return true;
}

// ---- write the solution ----
void Routing::write(string info)
{
	fdoutx << info.c_str() << endl;
	bestsolution.write(fdoutx);
	fdoutx << "\n\n\n\n";
}

// ---- show supply change list ----
/*
void Routing::show_SC()
{
	int i;
	printf("the left supply change events:\n");
	for (i = SCptr; i < totalSC; i++)
	{
		supplychange[i].show();
	}
}

// ---- show the rest events ----
void Routing::show_events(int estart)
{
	int i;
	printf("the left events\n");
	for (i = estart; i < Event::totalevent; i++) events[i].show();
}*/

// ---- get next event ----
// after file input, the events are sorted by time 
// and we will assume that the SC events are of no order, while they do not need 
/*
Event Routing::get_next_event()
{
	Event tmpevt;
	int tmpnid;
	int i;
	// if there is next upcoming event 
	if (EVTptr != total_time_based_event)
	{
		// if SC node between current time and upcoming event
		tmpnid = bestsolution.get_nearest_SC(current_time, events[EVTptr].time, tmpevt);
		if (tmpnid != -1)
		{
			// return nearest SC node
			shift_SC_head(tmpnid);
		}
		// else, no SC node in between
		{
			// return next upcoming event 
			tmpevt = events[EVTptr];
			EVTptr++;
		}
	}
	// else, no upcoming event
	else
	{
		// if still SC node 
		tmpnid = bestsolution.get_nearest_SC(current_time, -1, tmpevt);
		if (tmpnid != -1)
		{
			// return nearest SC node
			shift_SC_head(tmpnid);
		}
		// else
		{
			// return false, dynamic routing finished 
			tmpevt.isfea = false;
		}
	}
	return tmpevt;
}*/

// ---- shift certain SC to the head of the list ----
// prepare for the handler
/*
void Routing::shift_SC_head(int nid)
{
	SupplyChangeEvent tmpSCevt;
	int i;
	int j;
	for (i = 0; i < totalSC; i++)
	{
		if (supplychange[i].id == nid)
		{
			if (supplychange[i].ishandled == true)
			{
				printf("bad SC detected!\n");
				while (1);
			}
			else
			{
				tmpSCevt = supplychange[i];
				break;
			}
		}
	}
	for (j = i; j > 0; j++)
	{
		supplychange[j] = supplychange[j - 1];
	}
	supplychange[j] = tmpSCevt;
}*/

void Routing::write_generator_input(int caseid)
{
	ofstream fd_generator;
	char buff[64];
	printf("writing best solution ....\n");
	// system("pause");
	// sscanf(buff, "best_solution%d.txt", caseid);
	string file_name = "best_solution";
	_itoa(caseid, buff, 10);
	file_name += string(buff);
	file_name += ".txt";


	cout << file_name << endl;
	fd_generator.open(file_name);
	// fd_generator.open("best_solution.txt");
	bestsolution.write_generator_input(fd_generator);
	fd_generator.close();
}