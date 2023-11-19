#pragma once

#include <chrono>

class MyTimer
{
public:
	MyTimer() : timepoint(std::chrono::system_clock::time_point::min())
	{}
	
	void clear()
	{
		timepoint = std::chrono::system_clock::time_point::min();
	}

	bool isStarted() const
	{
		return (timepoint.time_since_epoch() != std::chrono::system_clock::duration(0));
	}

	void start()
	{
		timepoint = std::chrono::system_clock::now();
	}

	unsigned long getMs()
	{
		if (isStarted())
		{
			std::chrono::system_clock::duration diff;
			diff = std::chrono::system_clock::now() - timepoint;
			auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(diff);
			return static_cast<unsigned long>(durationMs.count());
		}

		return 0;
	}

private:

	std::chrono::system_clock::time_point timepoint;
};