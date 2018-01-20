// ---- main code for Dynamic-VRP ----
// ---- Francis YAO | 2016.08.24 | francisyao014@gmail.com ----
// this time make it more object oriented

#include "head.h"
#include "Routing.h"

// A routing object defined in global scope
Routing routing;

// some global variables
int Client::totalnode = 0;
int Vehicle::totalvehicle = 0;
int Event::totalevent = 0;

// solution
Solution bestsolution;
tabuhistory tahistory;

// tabu parameter 
double lumbda = 0.02;
double alpha = 1;
double beta = 1;
double gamma = 1;
double mdfactor = 2;

Vehicle vehicle[VEHICLE_LIMIT];
Client client[CLIENT_LIMIT];
Client depot;
// NOTE: costmatirx should always reffered by node id
double costmatrix[CLIENT_LIMIT][CLIENT_LIMIT];

int main(void)
{
	int totalcase;
	int i;
	printf("How many cases do you want to perform routing?");
	scanf("%d", &totalcase);
	for(i = 0; i < totalcase; i++)
	{
		routing.run(i);
	}
	system("pause");
	return 0;
}
