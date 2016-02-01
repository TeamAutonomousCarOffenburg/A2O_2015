#include <iostream>
#include <thread>

#include "EventLogger.h"
#include "LogReplayer.h"

#include "general/ComponentFactory.h"
#include "general/LogComponentFactory.h"
#include "general/A2ORuntime.h"

using namespace A2O;
using namespace std;

int main(int argc, char**argv){


	if(argc<2)
	{
		cout << "please give the file to be replayed as a cmd parameter" << endl;
		return -1;
	}

	// the following could be specified by command line
	// IComponentFactory::ConstPtr factory(new ComponentFactory());
	IComponentFactory::ConstPtr factory(new LogComponentFactory());
	bool realTime = false;
	unsigned int cycleTime = 20;

	// create and start runtime
	A2ORuntime* runtime = new A2ORuntime(factory);
	runtime->init();
	runtime->start();

	IPerception::Ptr perception = runtime->getPerception();
	
	string filename(argv[1]);
	LogReplayer* replay = new LogReplayer(filename, perception);
	replay->load();
	
	int time=replay->getFirstTime();

	while(replay->next(time))
	{
		runtime->update();

		if (realTime) {
			chrono::milliseconds dura(cycleTime);
			this_thread::sleep_for(dura);
		}
		time += cycleTime;
	}
}
