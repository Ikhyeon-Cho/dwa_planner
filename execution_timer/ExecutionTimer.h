#include <iostream>
#include <chrono>

class MicroTimer
{
public:
  MicroTimer() : start_time_(std::chrono::high_resolution_clock::now())
  {
  }

  void tic()
  {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  double toc(const std::string& message = "")
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
    double elapsed_time = duration.count();

    if (!message.empty())
    {
      std::cout << "[ " << message << " ]"
                << ": ";
    }
    std::cout << "takes " << elapsed_time << " microseconds" << std::endl;

    return elapsed_time;
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

class MilliTimer
{
public:
  MilliTimer() : start_time_(std::chrono::high_resolution_clock::now())
  {
  }

  void tic()
  {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  double toc(const std::string& message = "")
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);
    double elapsed_time = duration.count();

    if (!message.empty())
    {
      std::cout << "[ " << message << " ]"
                << ": ";
    }
    std::cout << "takes " << elapsed_time << " milliseconds" << std::endl;

    return elapsed_time;
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

class NanoTimer
{
public:
  NanoTimer() : start_time_(std::chrono::high_resolution_clock::now())
  {
  }

  void tic()
  {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

  double toc(const std::string& message = "")
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
    double elapsed_time = duration.count();

    if (!message.empty())
    {
      std::cout << "[ " << message << " ]"
                << ": ";
    }
    std::cout << "takes " << elapsed_time << " nanoseconds" << std::endl;

    ++n_;
    total_ += elapsed_time;
    std::cout << "total: " << n_ << " times with " << total_ / 1000 << " microsec" << std::endl;

    return elapsed_time;
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  int n_{ 0 };
  double total_{ 0 };
};