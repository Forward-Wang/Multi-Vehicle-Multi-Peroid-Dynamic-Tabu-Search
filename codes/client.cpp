#include "head.h"

// ---- initializer ----
Client::Client()
{
	// Basic variable -- read them from the file
	id = 0;
	type = 0;
	xcoordinate = 0;
	ycoordinate = 0;
	demand = 0;
	long_supply = 0;
	prsb_supply = 0;
	frequency = 7;
	opentime = 0;
	closetime = 120000;
	servicetime = 0;

	// auxiliary variables
	// load of node
	// long-time and perishable
	load = 0;
	long_load = 0;
	prsb_load = 0;
	// delivered demand
	long_delivered = 0;
	prsb_delivered = 0;

	fillrate = 0;
	// This is used both in route construction and clustering
	isinserted = false;

	newarrival = false;

	// cost = the time vehicle reaches the node + service time at that node
	cost = 0;

	// structural variable -- next position in the linked list
	next = 0;

	// distance from the center, for k-clustering algorithm
	distance = 0;

	// for stage iii --  fair allocation
	init_demand = 0;
	reduced_demand = 0;
}

// ---- a new arrival node ----
/*
Client::Client(NewArrivalEvent naevt)
{
	// Basic variable -- read them from the file
	id = -1;

	type = PICK;
	xcoordinate = naevt.x;
	ycoordinate = naevt.y;
	demand = 0;
	long_supply = naevt.lsupply;
	prsb_supply = naevt.psupply;
	frequency = 7;
	opentime = 0;
	closetime = 120000;
	servicetime = 0;

	// auxiliary variables
	// load of node
	// long-time and perishable
	load = 0;
	long_load = 0;
	prsb_load = 0;
	// delivered demand
	long_delivered = 0;
	prsb_delivered = 0;

	// This is used both in route construction and clustering
	isinserted = false;

	newarrival = true;

	// cost = the time vehicle reaches the node + service time at that node
	cost = 0;

	// structural variable -- next position in the linked list
	next = 0;

	// distance from the center, for k-clustering algorithm
	count_distance();

	// for stage iii --  fair allocation
	init_demand = 0;
	reduced_demand = 0;
}*/

// ---- count distance from the origin ----
void Client::count_distance()
{
	distance = sqrt(xcoordinate * xcoordinate + ycoordinate * ycoordinate);
}

// ---- print the client info ----
void Client::show() const
{
	if (type == PICK)
	{
		printf("%3d % *.2lf      % *.2lf      % *.2lf      ------------------------------     % *.2lf  % *.2lf      ----",
			id, 8, cost, 8, long_supply, 8, prsb_supply, 8, long_load, 8, prsb_load);
		if (newarrival) printf("    NA\n");
		else printf("\n");
	}
	else
	{
		printf("% 3d % *.2lf      --------------------  % *.2lf        % *.2lf       % *.2lf  % *.2lf  % *.2lf  % *.3lf\n",
			id, 8, cost, 8, demand, 8, long_delivered, 8, prsb_delivered, 8, long_load, 8, prsb_load, 8, fillrate);
	}
}

// ---- write client info into file ----
void Client::write(ofstream &fdoutx) const
{
	char outbuffer[200];
	if (type == PICK)
	{
		sprintf(outbuffer, "%3d % *.2lf      % *.2lf      % *.2lf      ------------------------------     % *.2lf  % *.2lf      ----",
			id, 8, cost, 8, long_supply, 8, prsb_supply, 8, long_load, 8, prsb_load);
		fdoutx << outbuffer;
		if (newarrival) fdoutx << "    NA\n";
		else fdoutx << "\n";
	}
	else
	{
		sprintf(outbuffer, "% 3d % *.2lf      --------------------  % *.2lf        % *.2lf       % *.2lf  % *.2lf  % *.2lf  % *.3lf\n",
			id, 8, cost, 8, demand, 8, long_delivered, 8, prsb_delivered, 8, long_load, 8, prsb_load, 8, fillrate);
		fdoutx << outbuffer;
	}
	return;
}

// ---- client comparasonm, for ranking ----
int clientcmp(const void *a, const void *b)
{
    Client *px = (Client *) a;
    Client *py = (Client *) b;
    Client x = *px;
    Client y = *py;
    if((x.ycoordinate >= 0) && (y.ycoordinate < 0))
    {
        // x in upper halfplane and y in lower halfplane
        return -1; // then x < y
    }
    else if((x.ycoordinate < 0) && (y.ycoordinate >= 0))
    {
        // y in upper halfplane and x in lower halfplane
        return 1; // then x > y
    }
    else
    {
        // x and y in the same halfplane, compare their cosine
        if((x.ycoordinate >= 0) && (y.ycoordinate >= 0))
        {
            if((x.xcoordinate / x.distance) > (y.xcoordinate / y.distance)) // cos(x) > cos(y)
            {
                return -1; // then x < y
            }
            else if((x.xcoordinate / x.distance) < (y.xcoordinate / y.distance)) // cos(x) > cos(y)
            {
                return 1; // then x > y
            }
            else // same cosine value
            {
                if(x.distance < y.distance) return -1;
                else return 1;
            }
        }
        else if((x.ycoordinate < 0) && (y.ycoordinate < 0))
        {
            if((x.xcoordinate / x.distance) > (y.xcoordinate / y.distance)) // cos(x) > cos(y)
            {
                return 1; // then x < y
            }
            else if((x.xcoordinate / x.distance) < (y.xcoordinate / y.distance)) // cos(x) > cos(y)
            {
                return -1; // then x > y
            }
            else // same cosine value
            {
                if(x.distance < y.distance) return -1;
                else return 1;
            }
        }
        else // something goes wrong
        {
            printf("step2, wrong comparasion!\n");
            x.show();
            y.show();
            exit(1);
        }
    }
}

// ---- deliver the load ----
// -- simply deliver it and do not care about violation
int Client::deliver(Client &clientto)
{
	// if enough, deliver as demand
	if (long_load + prsb_load >= clientto.demand)
	{
		if (prsb_load >= clientto.demand)
		{
			clientto.prsb_delivered = clientto.demand;
			clientto.prsb_load = prsb_load - clientto.prsb_delivered;
			clientto.long_delivered = 0;
			clientto.long_load = long_load;
		}
		else
		{
			clientto.prsb_delivered = prsb_load;
			clientto.prsb_load = 0;
			clientto.long_delivered = clientto.demand - clientto.prsb_delivered;
			clientto.long_load = long_load - clientto.long_delivered;	
		}
		clientto.load = clientto.long_load + clientto.prsb_load;
		clientto.calculate_fillrate();
		return 0;
	}
	// else, sufficient enough, deliver all you have
	else if (long_load + prsb_load >= clientto.reduced_demand)
	{
		clientto.prsb_delivered = prsb_load;
		clientto.prsb_load = 0;
		clientto.long_delivered = long_load;
		clientto.long_load = 0;
		clientto.load = clientto.long_load + clientto.prsb_load;
		clientto.calculate_fillrate();
		return 0;
	}
	// else, deliver reduced demand and make the load negative
	// deliver prsb_load first then long_load
	else
	{
		clientto.long_delivered = long_load;
		clientto.long_load = 0;
		clientto.prsb_delivered = clientto.reduced_demand - clientto.long_delivered;
		clientto.prsb_load = prsb_load - clientto.prsb_delivered;
		clientto.load = load - clientto.reduced_demand;
		clientto.calculate_fillrate();
		return 1;
	}
}

// ---- load the cargo ---
// -- load simply load it and do not care violation
int Client::loadcargo(Client &clientfrom, double capacity)
{
	long_load = clientfrom.long_load + long_supply;
	prsb_load = clientfrom.prsb_load + prsb_supply;
	load = long_load + prsb_load;
	if (load < capacity)
	{
		return 0;
	}
	else
	{
		// capacity exceeded, not feasible
		return 2;
	}
}

// ---- calculate fillrate ----
void inline Client::calculate_fillrate()
{
	fillrate = (prsb_delivered + long_delivered) / init_demand;
}

// ---- get distance of two nodes ----
double get_dist(Client &cltx, Client &clty)
{
	return sqrt(pow(cltx.xcoordinate - clty.xcoordinate, 2) + pow(cltx.ycoordinate - clty.ycoordinate, 2));
}

// ---- functions for stage v, Dynamic VRP ----

int Client::deliver_saturated(Client &clientto)
{
	// if enough, deliver as demand
	if (long_load + prsb_load >= clientto.demand)
	{
		if (prsb_load >= clientto.demand)
		{
			clientto.prsb_delivered = clientto.demand;
			clientto.prsb_load = prsb_load - clientto.prsb_delivered;
			clientto.long_delivered = 0;
			clientto.long_load = long_load;
		}
		else
		{
			clientto.prsb_delivered = prsb_load;
			clientto.prsb_load = 0;
			clientto.long_delivered = clientto.demand - clientto.prsb_delivered;
			clientto.long_load = long_load - clientto.long_delivered;
		}
		clientto.load = clientto.long_load + clientto.prsb_load;
		clientto.calculate_fillrate();
		return 0;
	}
	// else, sufficient enough, deliver all you have
	else if (long_load + prsb_load >= clientto.reduced_demand)
	{
		clientto.prsb_delivered = prsb_load;
		clientto.prsb_load = 0;
		clientto.long_delivered = long_load;
		clientto.long_load = 0;
		clientto.load = clientto.long_load + clientto.prsb_load;
		clientto.calculate_fillrate();
		return 0;
	}
	// else, still deliver all you have but return -1
	// deliver prsb_load first then long_load
	else
	{
		clientto.prsb_delivered = prsb_load;
		clientto.prsb_load = 0;
		clientto.long_delivered = long_load;
		clientto.long_load = 0;
		clientto.load = clientto.long_load + clientto.prsb_load;
		clientto.calculate_fillrate();
		return -1;
	}
}

// load whatever we can load
// if infeasibility, return -1
int Client::loadcargo_saturated(Client &clientfrom, double capacity)
{
	long_load = clientfrom.long_load + long_supply;
	prsb_load = clientfrom.prsb_load + prsb_supply;
	load = long_load + prsb_load;
	if (load < capacity)
	{
		return 0;
	}
	else
	{
		// capacity exceeded, not feasible
		// release some load, perishable load first
		if (prsb_load > load - capacity) prsb_load -= (load - capacity);
		else
		{
			long_load -= load - capacity - prsb_load;
			prsb_load = 0;
		}
		return -1;
	}
}

void Client::change_supply(Event *p)
{
	if (p->type == NC)
	{
		printf("node cancellation!\n");
		prsb_supply = 0;
		prsb_supply = 0;
	}
	else if (p->type == NV)
	{
		NegativeVariation *pnv = (NegativeVariation *)p;
		printf("negative variation!\n");
		printf("previous: prsb = %lf, long = %lf\n", prsb_supply, long_supply);
		prsb_supply -= pnv->perishable_decrease;
		long_supply -= pnv->longtime_decrease;
		printf("new:      prsb = %lf, long = %lf\n", prsb_supply, long_supply);
	}
	else if (p->type == PV)
	{
		PositiveVariation *ppv = (PositiveVariation *)p;
		printf("positive variation!\n");
		printf("previous: prsb = %lf, long = %lf\n", prsb_supply, long_supply);
		prsb_supply += ppv->perishable_increase;
		long_supply += ppv->longtime_increase;
		printf("new:      prsb = %lf, long = %lf\n", prsb_supply, long_supply);
	}
	return;
}

void Client::init(NodeArrival na_event)
{
	id = na_event.node_id;
	type = PICK;
	isinserted = false;
	newarrival = true;
	xcoordinate = na_event.x;
	ycoordinate = na_event.y;
	long_supply = na_event.lsupply;
	prsb_supply = na_event.psupply;
}