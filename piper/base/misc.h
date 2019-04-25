/**
 *  @file   misc.h
 *  @brief  miscellaneous functions
 *  @author Mustafa Mukadam
 *  @date   Dec 13, 2016
 **/

#ifndef MISC_H_
#define MISC_H_

#include <signal.h>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <gtsam/base/Vector.h>


namespace piper {

/**
 *  Convert std double vector type to gtsam Vector
 *
 *  @param v std double vector
 **/
static const gtsam::Vector getVector(const std::vector<double>& v)
{
  gtsam::Vector send(v.size());
  for (size_t i=0; i<v.size(); i++)
    send[i] = v[i];
  return send;
}

/**
 *  Convert gtsam Vector to std vector
 *
 *  @param v std double vector
 **/
static const std::vector<double> getStdVector(const gtsam::Vector& v)
{   
	std::vector<double> send;
	for(size_t i=0; i<v.size(); i++){
		send.push_back(v[i]);
	}
  return send;
}


/**
 *  Multiplu two quaternions
 *
 *  @param q, r std double vector of quaternions of the format [w,x,y,z]
 **/
static const std::vector<double> quatmultiply(const std::vector<double>& q, std::vector<double>& r) 
{
std::vector<double> vec;
double scalar;

vec = {q[0]*r[1] + r[0]*q[1] + q[2]*r[3]-q[3]*r[2],
        q[0]*r[2] + r[0]*q[2] + q[3]*r[1]-q[1]*r[3],
          q[0]*r[3] + r[0]*q[3] + q[1]*r[2]-q[2]*r[1]};

scalar = q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3];

std::vector<double> qout;
qout = vec;
qout.insert(qout.begin(), scalar);

return qout;
}

/**
 *  CTRL+C handling
 **/
static void sigintHandler(int sig)
{
  ROS_FATAL("Quitting...");
  ros::shutdown();
}


} // piper namespace

#endif
