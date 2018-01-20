#include "head.h"
#include "Solution.h"

extern Vehicle vehicle[VEHICLE_LIMIT];
extern Client client[CLIENT_LIMIT];
extern double costmatrix[CLIENT_LIMIT][CLIENT_LIMIT];
extern Solution bestsolution;

Client get_node_from_event(int node_id);

// ---- Initialise the vehicle ----
// -- copy all client from client[] to vehicle
void Vehicle::init(Client *client)
{
	memcpy(route, client, sizeof(route));
	head = 0;
	tail = 0;
	route[head].next = tail;
	len = 2;
	totalcost = 0;
	isfea_after_current = true;
}

// ---- Print the vehicle ---- 
void Vehicle::show() const
{
	int i, j;
	i = head;
	printf("---------------------------------------------------------\n");
	printf("    id: %d, len:%d, finalload:%lf, cost:%lf, p = %lf, q = %lf, t = %lf, dvrp_feasibility = %d, equity = %lf\n", 
		id, len, finalload, totalcost, p, q, t, isfea_after_current, equity);
	printf("----\n");
	printf("    id     cost   long_supply   prsb_supply  demand  long_delivered prsb_delivered long_load prsb_load fill_rate   new_arrival\n   ");
	for (j = 0; j < len - 1; j++)
	{
		route[i].show();
		printf("-> ");
		i = route[i].next;
	}
	// bug detect
	if (route[i].id != 0)
	{
		printf("id:%3d\tload:%ff\tcost:%f\n", route[i].id, finalload, totalcost);
		cout << "bad tail detected" << endl;
		while (1);
	}
	route[i].show();
}

// ---- write vehicle ----
void Vehicle::write(ofstream &fdoutx) const
{
	int i, j;
	i = head;
	char outbuffer[200];

	sprintf(outbuffer, "---------------------------------------------------------\n");
	fdoutx << outbuffer;
	sprintf(outbuffer, "    len:%d, load:%lf, cost:%lf, p = %lf, q = %lf, t = %lf, dvrp_feasibility = %d, equity = %lf\n", 
		len, finalload, totalcost, p, q, t, isfea_after_current, equity);
	fdoutx << outbuffer;
	sprintf(outbuffer, "----\n");
	fdoutx << outbuffer;
	sprintf(outbuffer, "    id     cost   long_supply   prsb_supply  demand  long_delivered prsb_delivered long_load prsb_load fill_rate   new_arrival\n");
	fdoutx << outbuffer;
	for (j = 0; j < len - 1; j++)
	{
		route[i].write(fdoutx);
		sprintf(outbuffer, " -> ");
		fdoutx << outbuffer;
		i = route[i].next;
	}
	// bug detect
	if (route[i].id != 0)
	{
		printf("id:%3d\tload:%ff\tcost:%f\n", route[i].id, finalload, totalcost);
		cout << "bad tail detected" << endl;
		while (1);
	}
	sprintf(outbuffer, "id:%3d\nlen:%d\tload:%lf\tcost:%lf\n", route[i].id, len, finalload, totalcost);
	fdoutx << outbuffer;
}



// ---- Feasibility check for route construction ----
int Vehicle::feasibility_check(int node, int position)
{
	int i, j, tmpret;
	i = position;
	j = node;

	// create a temporary copy of the present routelist and then check the feasibility
	Client tmplist[CLIENT_LIMIT];
	memcpy(tmplist, route, sizeof(route));

	tmplist[node].next = tmplist[position].next;
	tmplist[position].next = node;

	// update all the data along the list
	while (j != tail)
	{
		// This is a delivery node
		// deliver prsb_load first, then long_load
		if (tmplist[j].type == DELV)
		{
			tmpret = tmplist[i].deliver(tmplist[j]);
		}

		else
		{
			tmpret = tmplist[j].loadcargo(tmplist[i], capacity);
		}

		// if not feasible here, return directly
		if (tmpret != 0)
		{
			return tmpret;
		}

		// then calculate the cost
		tmplist[j].cost = tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id]
			+ tmplist[j].servicetime;

		if ((tmplist[j].opentime > tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id]) ||
			(tmplist[j].closetime < tmplist[j].cost))
		{
			// violate feasibility criteria 3
			if ((tmplist[j].opentime > tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id]))
			{
				printf("opentime > cost, time violation! opentime:%lf, cost:%lf\n",
					tmplist[j].opentime, tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id]);
				return 3;
			}
			else
			{
				printf("closetime < cost, time violation! closetime:%lf, cost:%lf\n", tmplist[j].closetime, tmplist[j].cost);
				return 4;
			}
		}
		i = tmplist[i].next; // note: this is equal to i = j
		j = tmplist[j].next;
	}

	// Coming to the end
	if ((tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id]) > closetime)
	{
		// violate feasibility criteria 4
		// Note here !
		// printf("Inserting node %d after node %d\n", tmplist[node].id, tmplist[position].id);
		printf("Finalcost > closetime, violate criteria 4! finalcost:%lf, closetime:%lf\n",
			tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id], closetime);
		// printf("id:%d", tmplist[i].id);
		return -1;
	}
	else
	{
		// satisfy all the feasibility criteria
		return 0;
	}
}

// ---- Cost calculate for route construction ----
double Vehicle::cost_calculate(int node, int position)
{
	int i, j;
	double tmpcost;
	Client tmplist[CLIENT_LIMIT];
	//printf("cost_calculate\n");

	i = position;
	j = node;
	memcpy(tmplist, route, sizeof(route));
	tmplist[node].next = tmplist[position].next;
	tmplist[position].next = node;

	// update all the data along the list
	while (j != tail)
	{
		tmplist[j].cost = tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id]
			+ tmplist[j].servicetime;
		i = tmplist[i].next; // note: this is equal to i = j
		j = tmplist[j].next;
	}
	// tmplist[j].cost = tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id] + tmplist[j].servicetime;
	tmpcost = tmplist[i].cost + costmatrix[tmplist[i].id][tmplist[j].id] + tmplist[j].servicetime;
	return tmpcost;
}

// ---- insert function for step3 ----
// insert node in position
void Vehicle::insert(int node, int position)
{
	int i, j;
	i = position;
	j = node;
	route[node].next = route[position].next;
	route[position].next = node;

	// client[node].isinserted = true;

	// update all the data along the list
	while (j != tail)
	{
		if (route[j].type == DELV)
		{
			route[i].deliver(route[j]);
		}
		else
		{
			route[j].loadcargo(route[i], capacity);
		}

		route[j].cost = route[i].cost + costmatrix[route[i].id][route[j].id]
			+ route[j].servicetime;
		i = route[i].next; // note: this is equal to i = j
		j = route[j].next;
	}
	// route[j].cost = route[i].cost + costmatrix[route[i].id][route[j].id] + route[j].servicetime;
	totalcost = route[i].cost + costmatrix[route[i].id][route[j].id];
	finalload = route[i].load;
	len++;
}

// ---- transformation ----
void Vehicle::linkedlist2array()
{
	Vehicle tmproute;
	head = 0;
	int i, j;
	i = head;
	for (j = 0; j < len - 1; j++)
	{
		tmproute.route[j] = route[i];
		tmproute.route[j].next = j + 1;
		i = route[i].next;
	}
	tmproute.route[j] = route[i];
	tail = j;
	memcpy(route, tmproute.route, sizeof(route));
}

// ---- insert a node to a vehicle ----
// simply insert and do not handle feasibility
// keep linked list structure
// modified to be compatable with sdvrp
void Vehicle::insert_i(int id, int position)
{
	// attach a new node with id at the end of the routelist and then maintain the linking structure
	Client tmpnd;
	int i;
	if (id < CLIENT_LIMIT)
	{
		for (i = 0; i < Client::totalnode; i++)
		{
			if (client[i].id == id)
			{
				tmpnd = client[i];
				break;
			}
		}
	}
	else
	{
		tmpnd = get_node_from_event(id);
	}
	route[len] = tmpnd;
	route[position].next = len;
	route[len].next = position + 1;
	len++;
}

// ---- insert, wrap insert_i, linkedlist2array, update ----
void Vehicle::insert_s(Client nd, int position)
{
	// insert_i(id, position);
	route[len] = nd;
	route[position].next = len;
	route[len].next = position + 1;
	len++;
	linkedlist2array();
	update();
}

// ---- kickout a node from a position ----
void Vehicle::kickout(int position)
{
	route[position - 1].next = position + 1;
	len--;
}


// ---- update the data in a route ----
// here we will assume all feasible? 
// -- we do not care feasible or not, just use feasibility_check function
// used in swap operator, as well as tabu search 
// based on linked list structure  -- but also work for array structure
// calculate equality 
void Vehicle::update()
{
	int i, j, k;
	int totaldelv = 0;
	double tmpfrsum = 0;
	double tmpavrg = 0;
	j = head;
	k = route[j].next;
	i = 1;
	p = 0;
	q = 0;
	t = 0;
	equity = 0;
	while (k != tail)
	{
		if (route[k].type == PICK)
		{
			route[k].loadcargo(route[j], capacity);
		}
		else
		{
			route[j].deliver(route[k]);
			tmpfrsum += route[k].fillrate;
			totaldelv += 1;
		}

		// load violation
		if (route[k].load < 0)
		{
			// printf("a load violate!\n");
			q -= route[k].load;
		}
		if (route[k].load > capacity)
		{
			// printf("a load violate!\n");
			p += route[k].load - capacity;
		}

		// cost of next node
		route[k].cost = route[j].cost + get_dist(route[j], route[k])/*costmatrix[route[j].id][route[k].id]*/ + route[k].servicetime;

		// Tour duration violation
		if (route[j].cost + get_dist(route[j], route[k])/*costmatrix[route[j].id][route[k].id]*/ < route[k].opentime)
		{
			// printf("a time violate!\n")
			t += route[k].opentime - (route[j].cost + get_dist(route[j], route[k])/*costmatrix[route[j].id][route[k].id]*/);
		}
		if (route[k].cost > route[k].closetime)
		{
			// printf("a time violate!\n");
			t += route[k].cost - route[k].closetime;
		}

		j = k;
		k = route[k].next;
		i++;
		// if(i == len - 1) break;
	}

	route[k].loadcargo(route[j], capacity);
	// k = tail
	route[k].cost = route[j].cost + get_dist(route[j], route[k])+ route[k].servicetime;

	// calculate the equity 
	tmpavrg = tmpfrsum / totaldelv;
	equity = 0;
	k = route[j].next;
	while (k != tail)
	{
		if (route[k].type == DELV)
		{
			equity += (route[k].fillrate - tmpavrg) * (route[k].fillrate - tmpavrg);
		}
		k = route[k].next;
	}
	equity = sqrt(equity / totaldelv);

	finalload = route[j].load;
	totalcost = route[j].cost + get_dist(route[j], route[head]) /*costmatrix[route[j].id][0]*/;
	if ((p == 0) && (q == 0) && (t == 0) && totalcost < closetime) isfea = true;
	else isfea = false;
	isfea_after_current = true;
	// printf("route update, finalcost = %lf\n", totalcost);
}

// for step4, 5, 6
// Based on linking structure
int Vehicle::feasibility_check()
{
	// note this function is based on the linking structure
	int j, k;
	j = head;
	k = route[j].next;
	while (k != tail)
	{
		//printf("fea mark\n");
		if ((route[k].load < 0) || (route[k].load > capacity))
		{
			// violate feasibility criteria 1, 2
			return 1;
		}
		if ((route[k].opentime > route[j].cost + costmatrix[route[j].id][route[k].id]) ||
			(route[k].closetime < route[k].cost))
		{
			// violate feasibility criteria 3
			return 1;
		}
		j = k;
		k = route[k].next;
	}
	if (route[j].cost + costmatrix[route[j].id][route[k].id] > closetime)
	{
		// violate feasibility criteria 4
		return -1;
	}
	else
	{
		// satisfy all the feasibility criteria
		return 0;
	}
}

// ---- intra-vehicle swap ----
void Vehicle::swap(int i, int j)
{
	// swap
	int k;
	Client tmp = route[i];
	route[i] = route[j];
	route[j] = tmp;
	// Maintain the linked list structure
	for (k = 0; k < len - 1; k++)
	{
		route[k].next = k + 1;
	}
}

// ---- swap operator at inter-vehicle level ----
bool swap_operator(Vehicle &vehiclei, Vehicle &vehiclej, int i, int j)
{
	// initialization
	// printf("swapping vehicle: i = %d, j = %d\n", i, j);
	Vehicle tmpvehiclei;
	Vehicle tmpvehiclej;
	Client tmpclientk;
	Client tmpclientl;
	int tmpnodek = -1;
	int tmpnodel = -1;
	double tmpcost = bestsolution.vehiclelist[i].totalcost + bestsolution.vehiclelist[j].totalcost;
	int k, l;

	// for every node k in route of vehicle i;
	for (k = 1; k < bestsolution.vehiclelist[i].len - 1; k++)
	{
		// and each node l in route of vehicle j
		for (l = 1; l < bestsolution.vehiclelist[j].len - 1; l++)
		{
			// printf("swapping: vehicle %d, position %d - vehicle %d, position %d\n", i , k, j, l);
			// Cvehicle_copy(vehicle[i], tmpvehiclei);
			// Cvehicle_copy(vehicle[j], tmpvehiclej);
			tmpvehiclei = bestsolution.vehiclelist[i];
			tmpvehiclej = bestsolution.vehiclelist[j];
			// store the intermediate node
			tmpclientk = tmpvehiclei.route[k];
			tmpclientl = tmpvehiclej.route[l];

			// make swap in position k, vehicle i
			tmpvehiclei.route[k] = tmpclientl;
			tmpvehiclei.route[k].next = k + 1;
			tmpvehiclei.update();

			// make swap in position l, vehicle j
			tmpvehiclej.route[l] = tmpclientk;
			tmpvehiclej.route[l].next = l + 1;
			tmpvehiclej.update();

			if ((tmpvehiclei.feasibility_check() == 0) && (tmpvehiclej.feasibility_check() == 0))
			{
				// printf("cost prev: %lf | now: %lf \n", tmpcost, tmpvehiclei.totalcost + tmpvehiclej.totalcost);
				if ((tmpvehiclei.totalcost + tmpvehiclej.totalcost) < tmpcost)
				{
					printf("find a swap: vehicle %d, position %d - vehicle %d, position %d\n", i, k, j, l);
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

					printf("cost reduced: %f -> %f\n\n", tmpcost, tmpvehiclei.totalcost + tmpvehiclej.totalcost);

					tmpnodel = l;
					tmpnodek = k;
					tmpcost = tmpvehiclei.totalcost + tmpvehiclej.totalcost;
				}
			}
		}
	}
	if (tmpnodel != -1)
	{
		//printf("Amoung all possible swaps, we choose the best: vehicle %d, position %d - vehicle %d, position %d\n", i, tmpnodek, j,tmpnodel);
		//printf("before\n");
		//printf("vehicle %d\n", i);
		//vehicle[i].vehicleroute.show();
		//printf("vehicle %d\n", j);
		//vehicle[j].vehicleroute.show();

		// store the cost before
		tmpcost = bestsolution.vehiclelist[i].totalcost + bestsolution.vehiclelist[j].totalcost;

		// store the intermediate node
		tmpclientk = bestsolution.vehiclelist[i].route[tmpnodek];
		tmpclientl = bestsolution.vehiclelist[j].route[tmpnodel];

		// make swap in position k, vehicle i
		bestsolution.vehiclelist[i].route[tmpnodek] = tmpclientl;
		bestsolution.vehiclelist[i].route[tmpnodek].next = tmpnodek + 1;
		// update load and cost
		bestsolution.vehiclelist[i].update();
		bestsolution.vehiclelist[i].linkedlist2array();

		// make swap in position l, vehicle j
		bestsolution.vehiclelist[j].route[tmpnodel] = tmpclientk;
		// maintain linking relation
		bestsolution.vehiclelist[j].route[tmpnodel].next = tmpnodel + 1;
		// update load and cost
		bestsolution.vehiclelist[j].update();
		bestsolution.vehiclelist[j].linkedlist2array();

		//printf("after:\n");
		//printf("vehicle %d\n", i);
		//vehicle[i].vehicleroute.show();
		//printf("vehicle %d\n", j);
		//vehicle[j].vehicleroute.show();
		//printf("cost reduce: %f -> %f\n", tmpcost, vehicle[i].vehicleroute.totalcost + vehicle[j].vehicleroute.totalcost);
		return true;
	}
	else
	{
		return false;
	}
}

// ---- rearrange operator ----
bool Vehicle::rearrange_operator()
{
	// auxiliary veriables
	Vehicle tmproute;
	int tmpswapk;
	int tmpswapj;
	int j, k;
	double tmpcost;

	if (len > 2)
	{
		// initialize
		// make the linkedlist transformed to be an array, while keep the linking relationship
		linkedlist2array();
		tmpswapj = -1;
		tmpswapk = -1;
		tmpcost = totalcost;

		// for every possible node in vehicle route
		// note that node start from 1, cos node 0 is the start node
		// note that the route end at index len - 1, then node j should end at len - 2, cos node k will end at lent - 1
		for (j = 1; j < len - 2; j++)
		{
			// and every possible other route in position
			for (k = j + 1; k < len - 1; k++)
			{
				// printf("try to swap %d and %d\n", j, k);
				// copy to temporary route
				// pay attention to the trick *this here!
				tmproute = *this;
				// Croute_copy(*this, tmproute);
				// swap the two terms i and j, and keep the linking relationship
				// note that the swap function is based on the array structure

				// printf("copy check\n");
				// tmproute.show();

				// printf("swap\n");
				tmproute.swap(j, k);

				// array2linkedlist; -- to be continue

				// printf("swap check\n");
				// tmproute.show();

				// update the cost and load
				// note that the update function is based on the linking structure -- modify this to make it based on the array structure?
				tmproute.update();

				// printf("update check\n");
				// if(j == 2 && k == 6) tmproute.show();

				if (tmproute.feasibility_check() == 0)
					// note this feasibility_check function is also based on the linking structure
				{
					// tmpvehicle.vehicleroute.cost_calculate();
					// note this cost_calculate function is a array version
					if (tmproute.totalcost < tmpcost)
					{
						//printf("find a swap! %d & %d \n",j, k);
						//cout << "before\n";
						//show();
						//cout << "after\n";
						//tmproute.show();
						//printf("cost reduced: %f -> %f\n", totalcost, tmproute.totalcost);
						tmpswapj = j;
						tmpswapk = k;
						tmpcost = tmproute.totalcost;
					}
				}
			}
		}
		if (tmpswapj != -1)
		{
			// an acceptable swap
			printf("Amoung all possible swaps, we choose the best: swap %d & %d \n", tmpswapj, tmpswapk);
			//cout << "before\n";
			//show();
			tmpcost = totalcost;
			swap(tmpswapj, tmpswapk);
			update();
			//cout << "after\n";
			//show();
			printf("cost reduce: %lf -> %lf\n", tmpcost, totalcost);
			linkedlist2array();
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

// ---- opt operator ----
// Based on the array structure
bool Vehicle::opt_operator(int i, int j)
{
	// i != j, i + 1 ！= j, j + 1 ! = i
	if (i == j || i + 1 == j || j + 1 == i)
	{
		printf("Illegal opt!\n");
		return false;
	}

	route[i].next = j;
	if (j == i + 2)
	{
		route[j].next = i + 1;
	}
	else
	{
		route[j].next = i + 2;
		route[j - 1].next = i + 1;
	}
	route[i + 1].next = j + 1;
	linkedlist2array();
	update();
	return true;
}

// ---- remove, wrap kickout, linkedlist2array and update ----
void Vehicle::remove(int position)
{
	kickout(position);
	linkedlist2array();
	update();
}

// ---- waste distribution ----
// how to do this?
bool Vehicle::distribute_waste()
{
	int j, k;
	double tmpwaste;
	double tmpmakeup;
	double tmpgap;
	int tmpdnodes;

	Vehicle tmpvehicle;
	tmpvehicle = *this;

	tmpwaste = finalload;
	tmpdnodes = 0;
	j = 0;
	// Calculate all delivery nodes
	while (j != tail)
	{
		if (tmpvehicle.route[j].type == DELV) tmpdnodes++;
		j = tmpvehicle.route[j].next;
	}

	// Calculate the temporary makeup to make compensation to each delivery node
	tmpmakeup = tmpwaste / tmpdnodes;
	k = 0;
	j = tmpvehicle.route[k].next;
	while (j != tail)
	{
		if (tmpvehicle.route[j].type == DELV)
		{
			tmpvehicle.route[k].deliver(tmpvehicle.route[j]);
			
			if (tmpvehicle.route[j].load < 0) return false;
			// fill the gap between demand and initial demand, normal case
			tmpgap = tmpvehicle.route[j].init_demand - tmpvehicle.route[j].demand;
			if (tmpmakeup <= tmpgap)
			{
				// enough load to deliver
				if (tmpvehicle.route[j].load >= tmpmakeup)
				{
					if (tmpvehicle.route[j].prsb_load >= tmpmakeup)
					{
						tmpvehicle.route[j].prsb_load -= tmpmakeup;
						tmpvehicle.route[j].prsb_delivered += tmpmakeup;
					}
					else
					{
						tmpvehicle.route[j].long_load -= tmpmakeup - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].long_delivered += tmpmakeup - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_load = 0;
					}
					tmpvehicle.route[j].load -= tmpmakeup;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpwaste -= tmpmakeup;
				}
				// load not enough, deliver all 
				else
				{
					tmpwaste -= tmpvehicle.route[j].load;

					tmpvehicle.route[j].long_delivered += tmpvehicle.route[j].long_load;
					tmpvehicle.route[j].long_load = 0;
					tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
					tmpvehicle.route[j].prsb_load = 0;
					tmpvehicle.route[j].load = 0;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpmakeup = tmpwaste / tmpdnodes;
				}
			}
			// fill the gap between demand and initial demand, if overflow
			else
			{
				// enough load
				if (tmpvehicle.route[j].load > tmpgap)
				{

					if (tmpvehicle.route[j].prsb_load >= tmpgap)
					{
						tmpvehicle.route[j].prsb_load -= tmpgap;
						tmpvehicle.route[j].prsb_delivered += tmpgap;
					}
					else
					{
						tmpvehicle.route[j].long_load -= tmpgap - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].long_delivered += tmpgap - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_load = 0;

					}
					tmpvehicle.route[j].load -= tmpgap;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpwaste -= tmpgap;
					// update tmpmakeup
					tmpmakeup = tmpwaste / tmpdnodes;
				}
				// load not enough, deliver all 
				else
				{
					tmpwaste -= tmpvehicle.route[j].load;

					tmpvehicle.route[j].long_delivered += tmpvehicle.route[j].long_load;
					tmpvehicle.route[j].long_load = 0;
					tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
					tmpvehicle.route[j].prsb_load = 0;
					tmpvehicle.route[j].load = 0;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpmakeup = tmpwaste / tmpdnodes;
				}
			}
		}
		else
		{
			tmpvehicle.route[j].loadcargo(tmpvehicle.route[k], capacity);
		}

		k = tmpvehicle.route[k].next;
		j = tmpvehicle.route[j].next;
	}
	tmpvehicle.route[j].loadcargo(tmpvehicle.route[k], capacity);
	*this = tmpvehicle;
	return true;
}

// ---- insert and return nird, node insert record ----
// only use this function for a tmpvehicle, but not a vehicle in a solution
/*
Vehicle::insert_na(Client clt, int position)
{
	NodeInsertRecord tmpnird;
	// relax time window
	 
	closetime = relaxted_closetime;
	insert_s(clt, position);

	tmpnird.vehicleid = this->id;
	tmpnird.position = position;
	tmpnird.equity = equity;
	tmpnird.cost = totalcost;
	if(isfea) tmpnird.valid = true;
	else tmpnird.valid = false;
	tmpnird.waste = finalload;

	return tmpnird;
}*/

// ---- join a route with another ----
// based on array structure
// the end of this vehicle, should be the start of tmpvehicle
bool Vehicle::join(Vehicle &tmpvehicle)
{
	// cannot join, something goes wrong!
	printf("joining vehicle %d and vehicle %d\n", id, tmpvehicle.id);
	// show_jn();
	// tmpvehicle.show();
	if (route[tail].id != tmpvehicle.route[0].id)
	{
		printf("Cannot join the two vehicles, something goes wrong!\n");
		while (1);
		return false;
	}
	int i, j;
	i = tail;
	j = 0;
	while (j != tmpvehicle.tail)
	{
		route[i] = tmpvehicle.route[j];
		route[i - 1].next = i;
		i++;
		j++;
	}
	route[i] = tmpvehicle.route[j];
	route[i - 1].next = i;
	tail = i;
	len = tail + 1;
	update();
	show();
	return true;
}

// ---- initialize vehicle with cluster ----
void Vehicle::init(Cluster &tmpcluster)
{
	tmpcluster.set_head_tail();

	memcpy(route, tmpcluster.clstclient, sizeof(route));

	int i;
	for (i = 0; i < tmpcluster.len; i++) tmpcluster.clstclient[i].isinserted = false;

	head = 0;
	tail = tmpcluster.len - 1;
	route[head].next = tail;
	tmpcluster.clstclient[head].isinserted = true;
	tmpcluster.clstclient[tail].isinserted = true;

	closetime = relaxted_closetime;
	closetime -= tmpcluster.time_reduce;
	len = 2;
	totalcost = 0;
	isfea_after_current = true;
}

// ---- Print the vehicle ---- 
void Vehicle::show_jn() const
{
	int i, j;
	i = head;
	printf("---------------------------------------------------------\n");
	printf("    id: %d\tlen:%d\tfinalload:%lf\tcost:%lf, p = %lf, q = %lf, t = %lf\n", id, len, finalload, totalcost, p, q, t);
	printf("----\n");
	printf("    id     cost   long_supply   prsb_supply  demand  long_delivered prsb_delivered long_load prsb_load fill_rate   new_arrival\n");
	for (j = 0; j < len - 1; j++)
	{
		printf("  ");
		route[i].show();
		printf("-> ");
		i = route[i].next;
	}
	route[i].show();
}


// ---- functions for stage v, DVRP ----


// get next visiting location of this vehicle
// following the same logic in function get_next_event
// need to consider start time, but keep the cost consistancy
// the start time is considered in construct on arrival function
int Vehicle::next_visiting_loc(double current_time)
{
	int j;
	int next_loc = -1;
	for (j = 0; j < len; j++)
	{
		if (route[j].cost > current_time)
		{
			next_loc = j;
			break;
		}
	}
	// if we have already finised routing within this vehicle, return -1
	return next_loc;
}

void Vehicle::init_na(double start_time, int id)
{
	// check whether the first node of this vehicle is the origin
	if (route[0].id != 0)
	{
		// bug detection
		printf("bug detected: the firt node of a new vehicle is not origin!");
		// write the current stage
		// TBC
		system("pause");
		exit(1);
	}
	else
	{
		route[1] = route[0];
		route[0].cost = start_time;
		route[0].next = 1;
		head = 0;
		tail = 1;
		len = 2;
	}
}

void Vehicle::insert_sdvrp(Client nd, int position)
{
	route[len] = nd;
	route[position].next = len;
	route[len].next = position + 1;
	len++;
	linkedlist2array();
	update_sdvrp();
}

void Vehicle::insert_na(Client nd, int position)
{
	// insert_i(id, position);
	nd.newarrival = true;
	route[len] = nd;
	route[position].next = len;
	route[len].next = position + 1;
	len++;
	linkedlist2array();
	update_dvrp();
}

// based on the linked list structure 
// maintain isfea_after_current
void Vehicle::update_dvrp()
{
	int i, j, k;
	int tmpret;
	j = head;
	k = route[j].next;
	i = 1;
	p = 0;
	q = 0;
	t = 0;
	isfea = true;
	isfea_after_current = true;
	equity = 0;
	while (k != tail)
	{
		if (route[k].type == PICK)
		{
			// route k load cargo based on route j
			tmpret = route[k].loadcargo_saturated(route[j], capacity);
		}
		else
		{
			// from route j, deliver to route k
			tmpret = route[j].deliver_saturated(route[k]);
		}
		if (tmpret != 0)
		{
			isfea = false;
			if (route[k].cost > *pcurrent_time) isfea_after_current = false;
		}

		// cost of next node
		route[k].cost = route[j].cost + get_dist(route[j], route[k]) + route[k].servicetime;

		// Tour duration violation
		if (route[j].cost + get_dist(route[j], route[k]) < route[k].opentime)
		{
			// printf("a time violate!\n")
			t += route[k].opentime - (route[j].cost + get_dist(route[j], route[k]));
			if (route[k].cost > *pcurrent_time) isfea_after_current = false;
		}
		if (route[k].cost > route[k].closetime)
		{
			// printf("a time violate!\n");
			t += route[k].cost - route[k].closetime;
			if (route[k].cost > *pcurrent_time) isfea_after_current = false;
		}

		j = k;
		k = route[k].next;
		i++;
		// if(i == len - 1) break;
	}
	route[k].cost = route[j].cost + get_dist(route[j], route[k]) + route[k].servicetime;
	// come to tail
	tmpret = route[k].loadcargo_saturated(route[j], capacity);
	if (tmpret != 0)
	{
		isfea = false;
		if (route[k].cost > *pcurrent_time) isfea_after_current = false;
	}

	finalload = route[j].load;
	prsb_finalload = route[j].prsb_load;
	long_finalload = route[j].long_load;
	totalcost = route[j].cost + get_dist(route[j], route[head]);

	if ((t != 0) || totalcost > closetime)
	{
		isfea = false;
		isfea_after_current = false;
	}
	// printf("route update, finalcost = %lf\n", totalcost);
}

// do not use saturated load& deliver
// only update nodes after next_visiting
// maintain isfea_after_next_visiting
void Vehicle::update_sdvrp()
{
	int i, j, k;
	int totaldelv = 0;
	double tmpfrsum = 0;
	double tmpavrg = 0;

	j = next_visiting_loc(*pcurrent_time);

	if (j == -1) return;
	if (j >= len - 1) return;

	// the same as previous
	k = route[j].next;
	i = 1;
	p = 0;
	q = 0;
	t = 0;
	equity = 0;
	while (k != tail)
	{
		if (route[k].type == PICK)
		{
			route[k].loadcargo(route[j], capacity);
		}
		else
		{
			route[j].deliver(route[k]);
			tmpfrsum += route[k].fillrate;
			totaldelv += 1;
		}

		// load violation
		if (route[k].load < 0)
		{
			// printf("a load violate!\n");
			q -= route[k].load;
		}
		if (route[k].load > capacity)
		{
			// printf("a load violate!\n");
			p += route[k].load - capacity;
		}

		// cost of next node
		route[k].cost = route[j].cost + get_dist(route[j], route[k])/*costmatrix[route[j].id][route[k].id]*/ + route[k].servicetime;

		// Tour duration violation
		if (route[j].cost + get_dist(route[j], route[k])/*costmatrix[route[j].id][route[k].id]*/ < route[k].opentime)
		{
			// printf("a time violate!\n")
			t += route[k].opentime - (route[j].cost + get_dist(route[j], route[k])/*costmatrix[route[j].id][route[k].id]*/);
		}
		if (route[k].cost > route[k].closetime)
		{
			// printf("a time violate!\n");
			t += route[k].cost - route[k].closetime;
		}

		j = k;
		k = route[k].next;
		i++;
		// if(i == len - 1) break;
	}

	route[k].loadcargo(route[j], capacity);
	// k = tail
	route[k].cost = route[j].cost + get_dist(route[j], route[k]) + route[k].servicetime;

	// calculate the equity 
	tmpavrg = tmpfrsum / totaldelv;
	equity = 0;
	k = route[j].next;
	while (k != tail)
	{
		if (route[k].type == DELV)
		{
			equity += (route[k].fillrate - tmpavrg) * (route[k].fillrate - tmpavrg);
		}
		k = route[k].next;
	}
	equity = sqrt(equity / totaldelv);

	finalload = route[j].load;
	totalcost = route[j].cost + get_dist(route[j], route[head]) /*costmatrix[route[j].id][0]*/;
	if (totalcost != route[k].cost)
	{
		printf("bug detected: cost incoherency!\n");
		system("pause");
	}
	if ((p == 0) && (q == 0) && (t == 0) && totalcost < closetime) isfea = true;
	else isfea = false;
	isfea_after_current = true;
	// printf("route update, finalcost = %lf\n", totalcost);
}

// perform rearrange operator for DVRP, modified from previous
// node before [the node right after current_time] (inclusive) should not be influenced by this function
// -- GOOD
void Vehicle::rearrange_operator_dvrp(double current_time)
{
	// auxiliary veriables
	Vehicle tmproute;
	int tmpswapk;
	int tmpswapj;
	int tmpstart = -1;
	int i, j, k;
	double tmpcost;

	if (len > 2)
	{
		// initialize
		// make the linkedlist transformed to be an array, while keep the linking relationship
		linkedlist2array();
		tmpswapj = -1;
		tmpswapk = -1;
		tmpcost = totalcost;

		// find out tmpstart
		// the next node right after current time
		for (i = 0; i < len; i++)
		{
			if (route[i].cost > current_time)
			{
				tmpstart = i + 1;
				break;
			}
		}
		// this route is finished
		if (tmpstart == -1)
		{
			tmpstart = CLIENT_LIMIT;
		}

		// for every possible node in vehicle route
		// note that node start from 1, cos node 0 is the start node
		// note that the route end at index len - 1, then node j should end at len - 2, cos node k will end at lent - 1
		for (j = tmpstart; j < len - 2; j++)
		{
			// and every possible other route in position
			for (k = j + 1; k < len - 1; k++)
			{
				// printf("try to swap %d and %d\n", j, k);
				// copy to temporary route
				// pay attention to the trick *this here!
				tmproute = *this;
				// swap the two terms i and j, and keep the linking relationship
				// note that the swap function is based on the array structure

				// printf("copy check\n");
				// tmproute.show();

				// printf("swap\n");
				tmproute.swap(j, k);

				// update the cost and load
				// note that the update function is based on the linking structure -- modify this to make it based on the array structure?
				tmproute.update_dvrp();

				// here we only accept feasible move 
				// note that this <isfea> is updated by the function update_dvrp()
				// we replace isfea by isfea_after_current
				if (tmproute.isfea_after_current == false)
					// note this feasibility_check function is also based on the linking structure
				{
					// tmpvehicle.vehicleroute.cost_calculate();
					// note this cost_calculate function is a array version
					if (tmproute.totalcost < tmpcost)
					{
						//printf("find a swap! %d & %d \n",j, k);
						//cout << "before\n";
						//show();
						//cout << "after\n";
						//tmproute.show();
						//printf("cost reduced: %f -> %f\n", totalcost, tmproute.totalcost);
						tmpswapj = j;
						tmpswapk = k;
						tmpcost = tmproute.totalcost;
					}
				}
			}
		}
		if (tmpswapj != -1)
		{
			// an acceptable swap
			printf("Amoung all possible swaps, we choose the best: swap %d & %d \n", tmpswapj, tmpswapk);
			//cout << "before\n";
			//show();
			tmpcost = totalcost;
			swap(tmpswapj, tmpswapk);
			update_dvrp();
			//cout << "after\n";
			//show();
			printf("cost reduce: %lf -> %lf\n", tmpcost, totalcost);
			linkedlist2array();
			return;
		}
		// no better found by this operator
		else
		{
			return;
		}
	}
	// the length of this vehicle is less than 2
	else
	{
		return;
	}
}

// dvrp version of opt operator
bool Vehicle::opt_operator_dvrp(int i, int j)
{
	// i != j, i + 1 ！= j, j + 1 ! = i
	if (i == j || i + 1 == j || j + 1 == i)
	{
		printf("Illegal opt!\n");
		return false;
	}

	route[i].next = j;
	if (j == i + 2)
	{
		route[j].next = i + 1;
	}
	else
	{
		route[j].next = i + 2;
		route[j - 1].next = i + 1;
	}

	route[i + 1].next = j + 1;
	linkedlist2array();
	update_dvrp();
	return true;
}

// modified from previous version
// maintain the variable isfea_after_current for future use
void Vehicle::waste_distribute_after_current()
{
	printf("before waste distribution:\n");
	show();
	int j, k;
	double tmpwaste;
	double tmpmakeup;
	double tmpgap;
	int tmpdnodes;
	int next_loc;
	int tmpret;

	Vehicle tmpvehicle;
	tmpvehicle = *this;

	tmpwaste = finalload;
	tmpdnodes = 0;
	next_loc = next_visiting_loc(*pcurrent_time);
	if (next_loc == -1) return;
	k = next_loc - 1;
	j = next_loc;
	// Calculate all delivery nodes
	while (j != tail)
	{
		if (tmpvehicle.route[j].type == DELV) tmpdnodes++;
		j = tmpvehicle.route[j].next;
	}

	// Calculate the temporary makeup to make compensation to each delivery node
	tmpmakeup = tmpwaste / tmpdnodes;
	j = next_loc;
	while (j != tail)
	{
		if (tmpvehicle.route[j].type == DELV)
		{
			tmpret = tmpvehicle.route[k].deliver_saturated(tmpvehicle.route[j]);
			if (tmpret < 0) tmpvehicle.isfea_after_current = false;

			// if (tmpvehicle.route[j].load < 0) return false;
			// fill the gap between demand and initial demand, normal case
			tmpgap = tmpvehicle.route[j].init_demand - tmpvehicle.route[j].demand;
			if (tmpmakeup <= tmpgap)
			{
				// enough load to deliver
				if (tmpvehicle.route[j].load >= tmpmakeup)
				{
					if (tmpvehicle.route[j].prsb_load >= tmpmakeup)
					{
						tmpvehicle.route[j].prsb_load -= tmpmakeup;
						tmpvehicle.route[j].prsb_delivered += tmpmakeup;
					}
					else
					{
						tmpvehicle.route[j].long_load -= tmpmakeup - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].long_delivered += tmpmakeup - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_load = 0;
					}
					tmpvehicle.route[j].load -= tmpmakeup;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpwaste -= tmpmakeup;
				}
				// load not enough, deliver all 
				else
				{
					tmpwaste -= tmpvehicle.route[j].load;

					tmpvehicle.route[j].long_delivered += tmpvehicle.route[j].long_load;
					tmpvehicle.route[j].long_load = 0;
					tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
					tmpvehicle.route[j].prsb_load = 0;
					tmpvehicle.route[j].load = 0;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpmakeup = tmpwaste / tmpdnodes;
				}
			}
			// fill the gap between demand and initial demand, if overflow
			else
			{
				// enough load
				if (tmpvehicle.route[j].load > tmpgap)
				{

					if (tmpvehicle.route[j].prsb_load >= tmpgap)
					{
						tmpvehicle.route[j].prsb_load -= tmpgap;
						tmpvehicle.route[j].prsb_delivered += tmpgap;
					}
					else
					{
						tmpvehicle.route[j].long_load -= tmpgap - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].long_delivered += tmpgap - tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
						tmpvehicle.route[j].prsb_load = 0;

					}
					tmpvehicle.route[j].load -= tmpgap;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpwaste -= tmpgap;
					// update tmpmakeup
					tmpmakeup = tmpwaste / tmpdnodes;
				}
				// load not enough, deliver all 
				else
				{
					tmpwaste -= tmpvehicle.route[j].load;

					tmpvehicle.route[j].long_delivered += tmpvehicle.route[j].long_load;
					tmpvehicle.route[j].long_load = 0;
					tmpvehicle.route[j].prsb_delivered += tmpvehicle.route[j].prsb_load;
					tmpvehicle.route[j].prsb_load = 0;
					tmpvehicle.route[j].load = 0;
					tmpvehicle.route[j].calculate_fillrate();

					tmpdnodes--;
					tmpmakeup = tmpwaste / tmpdnodes;
				}
			}
		}
		else
		{
			tmpret = tmpvehicle.route[j].loadcargo_saturated(tmpvehicle.route[k], capacity);
			if(tmpret < 0) tmpvehicle.isfea_after_current = false;
		}
		k = tmpvehicle.route[k].next;
		j = tmpvehicle.route[j].next;
	}

	tmpret = tmpvehicle.route[j].loadcargo_saturated(tmpvehicle.route[k], capacity);
	if (tmpret < 0) tmpvehicle.isfea_after_current = false;
	*this = tmpvehicle;
	finalload = route[j].load;
	printf("after waste distribution:\n");
	show();
	return;
}

// check feasibility after current time
int Vehicle::feasibility_check_sdvrp()
{
	// note this function is based on the linking structure, as well as the array structure
	// but this is maintained
	int j, k;
	int next_loc;
	next_loc = next_visiting_loc(*pcurrent_time);
	// this vehicle is finished after current time
	// viewed as feasible
	if (next_loc == -1)
	{
		return 0;
	}

	if (next_loc == tail)
	{
		if (route[next_loc].cost > closetime) return -1;
		else if (route[next_loc].load < 0 || route[next_loc].load > capacity) return 1;
		else return 0;
	}

	j = next_loc;
	k = route[j].next;
	while (k != tail)
	{
		//printf("fea mark\n");
		if ((route[k].load < 0) || (route[k].load > capacity))
		{
			// violate feasibility criteria 1, 2
			return 1;
		}
		if ((route[k].opentime > route[j].cost + get_dist(route[j], route[k])) ||
			(route[k].closetime < route[k].cost))
		{
			// violate feasibility criteria 3
			return 1;
		}
		j = k;
		k = route[k].next;
	}
	if (route[j].cost + get_dist(route[j], route[k]) > closetime)
	{
		// violate feasibility criteria 4
		return -1;
	}
	else
	{
		// satisfy all the feasibility criteria
		return 0;
	}
}

// deviation of fill rate
// based on the array structure
void Vehicle::calculate_equity()
{
	int i;
	double fillrate_sum = 0;
	double fillrate_average = 0;
	int tmp_total_deliver = 0;
	for (i = 1; i < len - 1; i++)
	{
		if (route[i].type == DELV)
		{
			fillrate_sum += route[i].fillrate;
			tmp_total_deliver++;
		}
	}
	equity = 0;
	fillrate_average = fillrate_sum / tmp_total_deliver;
	for (i = 1; i < len - 1; i++)
	{
		if (route[i].type == DELV)
		{
			equity += (route[i].fillrate - fillrate_average) * (route[i].fillrate - fillrate_average);
		}
	}
	equity = sqrt(equity);
	return;
}