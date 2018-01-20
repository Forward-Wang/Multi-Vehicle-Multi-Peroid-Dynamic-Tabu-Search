#ifndef EVENT_H_
#define EVENT_H_

#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <iostream>

using namespace std;

// event type definition
// node arrival
#define NA 0
// node cancellation
#define NC 1
// negative variation
#define NV 2
// positive variation
#define PV 3

// #define EPSILON 0.0001

class Event
{
	friend class Routing;
	friend class Solution;
	friend class EventComparePtr;
	friend class Client;
	friend int event_time_compare(const void *a, const void *b);
	friend void copy_event(Event **dst, Event *src);
	friend Client get_node_from_event(int node_id);
	// friend bool operator < (Event *a, Event *b);

protected:
	static int totalevent;
	int type;
	int node_id;
	// char type[500];
	double time;
	// bool isfea;
	bool ishandled;
	// bool valid;

public:
	Event(){}
	Event(int type, int node_id, double time)
	{
		this->time = time;
		this->type = type;
		this->node_id = node_id;
		ishandled = false;
		// valid = false;
	}
	virtual ~Event(){}
	virtual void show() const {}
	virtual void write(ofstream &fdoutx) const {}
};

class NodeArrival : public Event
{
	friend class Routing;
	friend class Solution;
	friend class Client;
	friend Client get_node_from_event(int node_id);
private:
	double x;
	double y;
	double lsupply;
	double psupply;
	bool successfully_handled;
public:
	NodeArrival(){}
	NodeArrival(int id, double time, double x, double y, double psupply, double lsupply) : Event(NA, id, time)
	{
		this->x = x;
		this->y = y;
		this->psupply = psupply;
		this->lsupply = lsupply;
		successfully_handled = true;
		return;
	}

	void show() const;
	void write(ofstream &fdoutx) const;
};

class NodeCancellation : public Event
{
	friend class Routing;
	friend class Solution;
	friend class Client;
public:
	NodeCancellation(){}
	NodeCancellation(int id, double time) : Event(NC, id, time){}

	void show() const;
	void write(ofstream &fdoutx) const;
};

class NegativeVariation : public Event
{
	friend class Routing;
	friend class Solution;
	friend class Client;
private:
	double perishable_decrease;
	double longtime_decrease;
public:
	NegativeVariation(){}
	NegativeVariation(int id, double time, double perishable_decrease, double longtime_decrease) : Event(NV, id, time)
	{
		this->perishable_decrease = perishable_decrease;
		this->longtime_decrease = longtime_decrease;
		return;
	}

	void show() const;
	void write(ofstream &fdoutx) const;
};

class PositiveVariation : public Event
{
	friend class Routing;
	friend class Solution;
	friend class Client;
public:
	double perishable_increase;
	double longtime_increase;
	PositiveVariation(){}
	PositiveVariation(int id, double time, double perishable_increase, double longtime_increase) : Event(PV, id, time)
	{
		this->perishable_increase = perishable_increase;
		this->longtime_increase = longtime_increase;
		return;
	}

	void show() const;
	void write(ofstream &fdoutx) const;
};

class NodeInsertRecord
{
public:
	int vehicleid;
	int position;
	// is valid?
	// route length not violate
	bool valid;
	double cost_increase;


	/*double waste;
	double cost;
	double equity;*/

	NodeInsertRecord();

	// bool operator > (const NodeInsertRecord &nird) const;
};

class EventComparePtr
{
public:
	bool operator() (Event *a, Event *b)
	{
		return a->time > b->time;
	}
};

#endif