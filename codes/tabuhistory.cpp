#include "head.h"
#include "tabu.h"

// tabu parameter
extern double alpha;
extern double beta;
extern double gamma;
extern double mdfactor;

tabuhistory::tabuhistory()
{
	total = 0;
	head = 0;
	tail = 0;
}

void tabuhistory::update()
{
	int i;
	for (i = 0; i < total; i++)
	{
		history[i].retaintime++;
	}
}

void tabuhistory::push(Shiftrecord tabuitem)
{
	// Bug detection codes
	if (tail == 8)
	{
		printf("tabu limit overflow!");
		while (1);
		exit(1);
	}

	history[total] = tabuitem;
	total++;
}

void tabuhistory::reset()
{
	total = 0;
	head = 0;
}

// If find record, return true, else return false
bool tabuhistory::check_record(Shiftrecord srd)
{
	int i;
	for (i = 0; i < total; i++)
	{
		if ((history[i].vehiclefromid == srd.vehicletoid) && (history[i].vehicletoid == srd.vehiclefromid) &&
			(history[i].nodeid == srd.nodeid))
		{
			// printf("Hit tabu history, jump!");
			// printf("  shift node %d from vehicle %d to vehicle %d\n", history[i].nodeid, history[i].vehiclefromid, history[i].vehicletoid);
			return true;
		}
	}
	return false;
}

// ---- reset tabu parameter ----
void tabu_para_reset()
{
	alpha = 1;
	beta = 1;
	gamma = 1;
}

// ---- update tabu parameter ----
void tabu_para_update(bool isfea)
{
	// if feasible 
	if (isfea)
	{
		alpha /= mdfactor;
		beta /= mdfactor;
		gamma /= mdfactor;
	}
	else
	{
		alpha *= mdfactor;
		beta *= mdfactor;
		gamma *= mdfactor;
	}
}