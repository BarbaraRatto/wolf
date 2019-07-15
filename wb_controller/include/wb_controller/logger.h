#ifndef LOGGER_H
#define LOGGER_H

#include <ros/ros.h>
#include <wb_controller/publishers.h>

namespace wb_controller
{

class Logger
{
public:

    static Logger& getLogger()
    {
        static Logger logger;
        return logger;
    }

    template <typename data_t>
    void addPublisher(const std::string& name, const data_t& x)
    {
        publishers_->addPublisher(name,&x);
    }

    void publish(const ros::Time& /*time*/)
    {
        publishers_->publishAll();
    }

private:

  Logger()
  {
      ros::NodeHandle logger_nh("logger");
      publishers_.reset(new RealTimePublishers(logger_nh));
  }

  //~Logger()

  Logger(const Logger&)= delete;
  Logger& operator=(const Logger&)= delete;

  RealTimePublishers::Ptr publishers_;

};


} // namespace


#endif
