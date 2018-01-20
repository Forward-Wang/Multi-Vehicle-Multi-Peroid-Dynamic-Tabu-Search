#ifndef TABU_H_
#define TABU_H_

#include "head.h"


// ---- shiftrecord class for tabu search ----
class Shiftrecord
{
public:
	int nodepos;
	int nodeid;
	int position;
	int vehiclefromid;
	int vehicletoid;

	// the time of a node retained in the search
	int retaintime;

	// variable for checking whether there is valid shift operations
	bool isvalid;

	// default constructor
	Shiftrecord()
	{
		// next = NULL;
		retaintime = 0;
		isvalid = true;
	}

	Shiftrecord(int nodeid, int vehiclefromid, int vehicletoid);
	~Shiftrecord()
	{
		// Clean the memory
		// if(next != NULL) delete next;
	}
};

// ---- tabu history class ----
class tabuhistory
{
public:
	Shiftrecord history[TABUTENURE];

	int total;
	int head;
	int tail;
	//int level;

	tabuhistory();

	~tabuhistory(){}

	// Push in a new item
	void push(Shiftrecord tabuitem);

	// Update the retaining time of all the retained node
	void update();

	// Reset the history
	void reset();

	// Check the history
	bool check_record(Shiftrecord srd);

};

void tabu_para_reset();
void tabu_para_update(bool isfea);

#endif 