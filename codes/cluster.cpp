#include "cluster.h"
#include "Routing.h"

extern Solution bestsolution;
extern Client client[CLIENT_LIMIT];
extern Client depot;

// ---- build initial cluster ----
bool Routing::build_cluster_init()
{
	
	int i, j, k;
	int currentc, currentp, currentd;
	Client depot;
	int totnp, totnd;
	bool ret = true;
	j = 0, k = 0;

	// Initialize the client cluster
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		cluster[i].len = 0;
		cluster[i].totalpickdemand = 0;
		cluster[i].totaldelvdemand = 0;
	}

	// sort the clients
	qsort(client, Client::totalnode, sizeof(Client), clientcmp);

	// Devide the clients into pickups and deliverys
	for (i = 0; i < Client::totalnode; i++)
	{
		if (client[i].id == 0)
		{
			continue;
		}
		else if (client[i].type == PICK)
		{
			pickups[j] = client[i];
			pickups[j].isinserted = false;
			j++;
		}
		else
		{
			deliverys[k] = client[i];
			deliverys[k].isinserted = false;
			k++;
		}
	}

	printf("Total %d pickups, %d deliveries.\n", j, k);
	totnp = j;
	totnd = k;

	currentc = 1;
	currentp = 0;
	currentd = 0;
	printf("Clustering start:\n");
	cluster[currentc].show();
	while (1)
	{
		// All node clustered
		if (currentd == totnd && currentp == totnp) break;
		// Delivery nodes use up, only insert pickup nodes
		else if (currentd == totnd)
		{
			cluster[currentc].push(pickups[currentp]);
			pickups[currentp].isinserted = true;
			currentp++;
		}
		// Pickup nodes use up, only insert delivery nodes, and detect whether these nodes are already inserted
		// Here we should detect if total pickup larger than delivery, in these case we may leave some nodes unclustered
		else if (currentp == totnp)
		{
			if (deliverys[currentd].isinserted == true) currentd++;
			else if (cluster[currentc].totalpickdemand + cluster[currentc].totaldelvdemand + deliverys[currentd].demand >= 0)
			{
				cluster[currentc].push(deliverys[currentd]);
				deliverys[currentd].isinserted = true;
				currentd++;
			}
			// Not enough pickup for delivery, in this case we will end up nodes unclustered.
			else
			{
				printf("A node unclustered!\n");
				currentd++;
			}
		}
		// All two types available, always try to insert delivery nodes first
		else
		{
			if (deliverys[currentd].isinserted == true)
			{
				currentd++;
			}
			else if (cluster[currentc].totalpickdemand >= cluster[currentc].totaldelvdemand + deliverys[currentd].demand)
			{
				cluster[currentc].push(deliverys[currentd]);
				deliverys[currentd].isinserted = true;
				currentd++;
			}
			else
			{
				cluster[currentc].push(pickups[currentp]);
				pickups[currentp].isinserted = true;
				currentp++;
			}
			// If the cluster is large enough, cut it and try next
			if (cluster[currentc].len > 25)
			{
				for (j = 0; j < 5; j++)
				{
					if (currentd + j >= totnd) break;
					if (cluster[currentc].totalpickdemand >= cluster[currentc].totaldelvdemand + deliverys[currentd + j].demand)
					{
						cluster[currentc].push(deliverys[currentd + j]);
					}
				}
				currentd += j;
				cluster[currentc].head = depot;
				cluster[currentc].tail = depot;

				printf("A cluster ended!\n");
				cluster[currentc].show();
				currentc++;
			}
		}
	}
	printf("currentd = %d, currentp = %d\n", currentd, currentp);

	// detect whether too many clusters
	if (currentc > Vehicle::totalvehicle)
	{
		printf("Clustering failed: not enough vehicle!");
		return false;
	}

	// detect whether node left unserved
	printf("Clustering funished!\n");
	for (i = 1; cluster[i].len > 0; i++)
	{
		printf("cluster %d\n", i);
		cluster[i].show();
	}
	printf("Here are nodes left unclustered:\n");
	for (i = 0; i < totnp; i++)
	{
		if (pickups[i].isinserted == false)
		{
			pickups[i].show();
			ret = false;
		}
	}
	for (i = 0; i < totnd; i++)
	{
		if (pickups[i].isinserted == false)
		{
			deliverys[i].show();
			ret = false;
		}
	}
	return ret;
}

// ---- event comes, build cluster ----
// -- input: event
// -- do: change the clusters in Routing class
// detect the states of vehicles when the event happens
// build clusters accordingly 
/*
bool Routing::build_cluster_at_event(SupplyChangeEvent tmpevent)
{
	// tmpsolution = bestsolution;

	int i, j, cutposition;
	// for every vehicle, come to that time 
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		cluster[i].erase();
		bestsolution.vehiclelist[i].show();
		if (bestsolution.vehiclelist[i].len <= 2) continue;
		// find the first node that the vehicle will reach after the event happens
		for (j = bestsolution.vehiclelist[i].route[0].next;
			bestsolution.vehiclelist[i].route[j].cost < tmpevent.time;
			j = bestsolution.vehiclelist[i].route[j].next)
		{
			if (j == bestsolution.vehiclelist[i].tail)
			{
				printf("When the supply change, the vehicle has finished its task!\n");
				return false;
			}

			if (bestsolution.vehiclelist[i].route[j].id == tmpevent.id)
			{
				printf("When the supply changes, the vehicle has already pass the node!\n");
				printf("at time %lf, vehicle pass the node, at time %lf, the node changes its supply\n",
					bestsolution.vehiclelist[i].route[j].cost, tmpevent.time);
				printf("we cannot re-construct on site, so we will build the whole route again!\n");
				return false;
			}
		}

		// cut tmpsolution
		cutposition = j;

		printf("cut vehicle %d from node %d, time %lf\n", i,
			bestsolution.vehiclelist[i].route[j].id, bestsolution.vehiclelist[i].route[j].cost);
		cluster[i].time_reduce = bestsolution.vehiclelist[i].route[j].cost;

		cluster[i].head = bestsolution.vehiclelist[i].route[j];
		j = bestsolution.vehiclelist[i].route[j].next;
		// start from this node, construct the cluster
		for (; j != bestsolution.vehiclelist[i].tail; j = bestsolution.vehiclelist[i].route[j].next)
		{
			if (bestsolution.vehiclelist[i].route[j].id == tmpevent.id)
			{
				bestsolution.vehiclelist[i].route[j].long_supply = tmpevent.newlsupply;
				bestsolution.vehiclelist[i].route[j].prsb_supply = tmpevent.newpsupply;
			}
			cluster[i].push(bestsolution.vehiclelist[i].route[j]);
		}
		cluster[i].tail = bestsolution.vehiclelist[i].route[j];
		

		// cut the best solution
		bestsolution.vehiclelist[i].tail = cutposition;
		bestsolution.vehiclelist[i].len = cutposition + 1;

		printf("cluster %d construction finished!\n", i);
		cluster[i].show();
	}

	// copy tmpsolution to bestsolution, which means that bestsolution is been cut here
	// bestsolution = tmpsolution;

	// latter we will call build_solution_clustering() to build tmpsolution again, and then join the two;
	return true;
}*/

/*
bool Routing::build_cluster_at_event(NodeDisappearEvent tmpevent)
{
	int i, j, cutposition;
	// for every vehicle, come to that time 
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		cluster[i].erase();
		if (bestsolution.vehiclelist[i].len <= 2) continue;
		// find the first node that the vehicle will reach after the event happens
		for (j = bestsolution.vehiclelist[i].route[0].next;
			bestsolution.vehiclelist[i].route[j].cost < tmpevent.time;
			j = bestsolution.vehiclelist[i].route[j].next)
		{
			if (j == bestsolution.vehiclelist[i].tail)
			{
				printf("When the node disappear, the vehicle has finished its task!\n");
				return false;
			}
			if (bestsolution.vehiclelist[i].route[j].id == tmpevent.id)
			{
				printf("When the node disappear, the vehicle has already pass the node!\n");
				printf("at time %lf, vehicle pass the node, at time %lf, the disappear\n",
					bestsolution.vehiclelist[i].route[j].cost, tmpevent.time);
				printf("we cannot re-construct on site, so we will build the whole route again!\n");
				return false;
			}
		}

		// cut tmpsolution
		cutposition = j;

		printf("cut vehicle %d from node %d, time %lf\n", i,
			bestsolution.vehiclelist[i].route[j].id, bestsolution.vehiclelist[i].route[j].cost);
		cluster[i].time_reduce = bestsolution.vehiclelist[i].route[j].cost;

		cluster[i].head = bestsolution.vehiclelist[i].route[j];
		j = bestsolution.vehiclelist[i].route[j].next;
		// start from this node, construct the cluster
		for (; j != bestsolution.vehiclelist[i].tail; j = bestsolution.vehiclelist[i].route[j].next)
		{
			if (bestsolution.vehiclelist[i].route[j].id == tmpevent.id)
			{
				continue; // note this line is the only difference from supply change event 
			}
			cluster[i].push(bestsolution.vehiclelist[i].route[j]);
		}
		cluster[i].tail = bestsolution.vehiclelist[i].route[j];

		// cut the best solution
		bestsolution.vehiclelist[i].tail = cutposition;
		bestsolution.vehiclelist[i].len = cutposition + 1;

		printf("cluster %d construction finished!\n", i);
		cluster[i].show();
	}
	return true;
}*/


// ---- use clustering to build initial solution ---- 
// make sure that for every cluster, the head and tail of that cluster are already been inserted 
// we will build the solution into tmpsolution
bool Routing::build_solution_clustering()
{
	printf("build solution using clustering....\n");
	int i, j, k, l;
	int tmpposition;
	double tmpmincost;
	double tmpcost;
	int feasibilityret;
	int jumpcnt;
	bool terminateflag;
	bool is_node_left = false;

	// for every vehicle
	for (i = 1; i <= Vehicle::totalvehicle; i++)
	{
		printf("Constructing route %d using cluster %d...\n", i, i);
		// cluster[i].show();
		// Construct the route for vehicle i
		if (cluster[i].len > 0)
		{

			// Prepare the vehicle, initialize with cluster i
			tmpsolution.vehiclelist[i] = bestsolution.vehiclelist[i];
			tmpsolution.vehiclelist[i].init(cluster[i]);
			jumpcnt = 0;

			// try every node that has not been inserted
			for (j = 1; j < cluster[i].len; j++)
			{
				// to insert it in every possible route location in vehicle[i]'s route
				// terminal condition: 1> no avaiable node 2> vehicle time limit use up
				if (cluster[i].clstclient[j].isinserted == true) // this is the flag to show whether this node is inserted
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
					// printf("trying to insert node %d to vehicle %d, position %d\n", cluster[i].client[j].id, i, l);
					feasibilityret = tmpsolution.vehiclelist[i].feasibility_check(j, k);
					// here are the return value of feasibility_check function:
					// 0: feasible; 1: violate feasibility criteria 1,2,3; -1: violate feasibility criteria4

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

					else if (feasibilityret == 1) // violate feasibility criteria 1,2,3
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
						//printf("terminate detected!\n");
						//vehicle[i].vehicleroute.show();
						//printf("node and position failed to insert: %d, %d", j, k);
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
					// printf("node %d jumped\n", cluster[i].client[j].id);
				}
				else
				{
					// insert and update
					tmpsolution.vehiclelist[i].insert(j, tmpposition);
					cluster[i].clstclient[j].isinserted = true;

					// print intermediate state
					// printf("a new insert!\n");
					// tmpsolution.vehiclelist[i].show();
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
			tmpsolution.vehiclelist[i].linkedlist2array();
			tmpsolution.vehiclelist[i].show();
			printf("\n");
			// Detect the nodes left, if any, push to next cluster
			for (j = 1; j < cluster[i].len; j++)
			{
				if (cluster[i].clstclient[j].isinserted == false)
				{
					printf("In current route there is a node left, push to next cluster!\n");
					cluster[i].clstclient[j].show();
					cluster[i + 1].push(cluster[i].clstclient[j]);
					cluster[i].pop(j);
					j--;
				}
			}
		}
		else
		{
			break;
		}
	}
	printf("Here are all nodes left:\n");
	for (i = 1; i < Vehicle::totalvehicle; i++)
	{
		for (j = 0; j < cluster[i].len; j++)
		{
			if (cluster[i].clstclient[j].isinserted == false)
			{
				printf("id: %d, demand: %lf\n", cluster[i].clstclient[j].id, cluster[i].clstclient[j].demand);
				is_node_left = true;
			}
		}
	}
	tmpsolution.show();
	tmpsolution.eval();

	return !is_node_left;
}

// ---- use clustering to build initial solution ---- 
bool Routing::build_solution_clustering_init()
{
	// build the initial clusters
	build_cluster_init();
	// based on clusters, build solution, store in tmpolution
	build_solution_clustering();
	// copy tmpsolution to bestsolution
	bestsolution = tmpsolution;
	return true;
}

// ---- push into cluster ----
void Cluster::push(Client clt)
{
	clstclient[len] = clt;
	// printf("pushing to cluster: id = %d, type = %d, demand = %lf\n", clt.id, clt.type, clt.demand);
	if (clt.type == PICK)
	{
		totalpickdemand += clt.long_supply + clt.prsb_supply;
	}
	else
	{
		totaldelvdemand += clt.demand;
	}
	// printf("push begin, len = %d\n", len);
	len++;
	// printf("push finished, len = %d\n", len);
}

// ---- pop one node from cluster ----
void Cluster::pop(int pos)
{
	int i;
	if (clstclient[pos].type == PICK) totalpickdemand -= clstclient[pos].long_supply + clstclient[pos].prsb_supply;
	else totaldelvdemand -= clstclient[pos].demand;
	for (i = pos; i < len - 1; i++)
	{
		clstclient[i] = clstclient[i + 1];
	}
	len--;
}

// ---- print out the cluster ----
void Cluster::show() const
{
	int i = 0;
	for (i = 0; i < len; i++)
	{
		clstclient[i].show();
	}
	printf("len = %d, total pickup = %lf, total delivery = %lf\n", len, totalpickdemand, totaldelvdemand);
}

// ---- erase a cluster ----
void Cluster::erase()
{
	len = 0;
	totaldelvdemand = 0;
	totalpickdemand = 0;
	head = depot;
	tail = depot;
	return;
}

// ---- constructor ----
Cluster::Cluster()
{
	len = 0;
	totaldelvdemand = 0;
	totalpickdemand = 0;
	
	head = depot;
	tail = depot;
}

void Cluster::set_head_tail()
{
	int i;
	for (i = len; i >= 1; i--)
	{
		clstclient[i] = clstclient[i - 1];
	}
	clstclient[0] = head;
	clstclient[len + 1] = tail;
	len += 2;
}