#include "event.h"
#include "head.h"

void NodeArrival::show() const
{
	printf("NA | %d %lf %lf %lf %lf %lf\n", node_id, time, x, y, lsupply, psupply);
	return;
}

void NodeArrival::write(ofstream &fdoutx) const
{
	char buff[512];
	sprintf(buff, "NA | %d %lf %lf %lf %lf %lf\n", node_id, time, x, y, lsupply, psupply);
	fdoutx << buff;
	return;
}

void NodeCancellation::show() const
{
	printf("NC | %d %lf\n", node_id, time);
	return;
}

void NodeCancellation::write(ofstream &fdoutx) const
{
	char buff[512];
	sprintf(buff, "NC | %d %lf\n", node_id, time);
	fdoutx << buff;
	return;
}

void NegativeVariation::show() const
{
	printf("NV | %d %lf %lf %lf\n", node_id, time, perishable_decrease, longtime_decrease);
	return;
}

void NegativeVariation::write(ofstream &fdoutx) const
{
	char buff[512];
	sprintf(buff, "NV | %d %lf %lf %lf\n", node_id, time, perishable_decrease, longtime_decrease);
	fdoutx << buff;
	return;
}

void PositiveVariation::show() const
{
	printf("PV | %d %lf %lf %lf\n", node_id, time, perishable_increase, longtime_increase);
	return;
}

void PositiveVariation::write(ofstream &fdoutx) const
{
	char buff[512];
	sprintf(buff, "PV | %d %lf %lf %lf\n", node_id, time, perishable_increase, longtime_increase);
	fdoutx << buff;
	return;
}


NodeInsertRecord::NodeInsertRecord()
{
	vehicleid = -1;
	position = -1;
	// is valid?
	// route length not violate
	valid = false;
	cost_increase = 0;
}


