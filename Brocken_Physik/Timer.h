#ifndef __CUDACC__

#ifndef _TIMER_H_
#define _TIMER_H_


#include <boost\chrono.hpp>
#include "types.h"


using namespace boost::chrono;


class Timer{
private:
	steady_clock::time_point last, now;

public:
	Timer():last(steady_clock::now()){}

	void start(){
		last = steady_clock::now();
	}

	f64 tick(){
		now = steady_clock::now();
		steady_clock::duration dif = now - last;
		last = now;
		return static_cast<f64>(dif.count()) * steady_clock::period::num / steady_clock::period::den;
		return .01667f;
	}

	steady_clock::time_point getLast() const {
		return last;
	}
};


#endif


#else

class Timer{};

#endif