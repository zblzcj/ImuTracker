#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <errno.h>
#include <dirent.h>
#include <sys/types.h>


#include "lib-eigen/Eigen/Geometry"
#include "imu_tracker.h"

# define M_PI           3.14159265358979323846  /* pi */
# define M_G            9.80665                 /* g  */
//using namespace std;

int getdir(std::string dir, std::vector<std::string> &files)
{
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
    std::cout << "Error(" << errno << ") opening " << dir << std::endl;
    return errno;
  }

  while ((dirp = readdir(dp)) != NULL) {
    files.push_back(std::string(dirp->d_name));
  }
  closedir(dp);
  return 0;
}

const double r_to_d = 180 / M_PI;

int main(int argc, char * argv[])
{
  // Input imu data file names
  std::string dir("./kitti_0926_0117/data/");
  std::vector<std::string> files;
  getdir(dir, files);
  std::sort(files.begin(), files.end());

  // Input timestamp file
  std::ifstream infile_time;
  infile_time.open("./kitti_0926_0117/timestamps.txt");

  // Output file
  std::ofstream outfile_gps;
  outfile_gps.open("kitti_gps_0926_0117.txt");
  outfile_gps << std::fixed << std::setprecision(10);
  std::ofstream outfile_imu;
  outfile_imu.open("kitti_imu_0926_0117.txt");
  outfile_imu << std::fixed << std::setprecision(10);
  
  //for (unsigned int i = 2; i < files.size(); i++) {
  for (unsigned int i = 2; i < 5; i++) { 
    // Fetch imu data
    std::ifstream infile_imu;
    std::string filename = dir + files[i];
    infile_imu.open(filename);
    std::string line;
    std::getline(infile_imu, line);
    std::vector<std::string> tokens;
    std::istringstream iss(line);
    for (std::string s; iss >> s;)
      tokens.push_back(s);
    infile_imu.close();
    Eigen::Vector3d linear_acceleration(std::stod(tokens[11]) / M_G,
      std::stod(tokens[12]) / M_G, std::stod(tokens[13]) / M_G);
    Eigen::Vector3d angular_velocity(std::stod(tokens[17]),
      std::stod(tokens[18]), std::stod(tokens[19]));

    // Fetch time stamp
    std::getline(infile_time, line);
    std::string t_str = line.substr(line.find(" ") + 1);
    double time = std::stoi(t_str.substr(0, 2)) * 3600. + 
      std::stoi(t_str.substr(3, 2)) * 60. +
      std::stod(t_str.substr(6, 12)) * 1.;
  }

  infile_time.close();
  outfile_gps.close();
  outfile_imu.close();

  /*
  const double r_to_d = 180 / M_PI;
  double time = res->getDouble("server_time");
  //Eigen::Vector3d ini_orientation(res->getDouble("anglex") / r_to_d,
    //res->getDouble("angley") / r_to_d, res->getDouble("anglez") / r_to_d);
  //ImuTracker imu_tracker(1e1, time, ImuTracker::toQuaternion(ini_orientation));
  ImuTracker imu_tracker(1e1, time, Eigen::Quaterniond::Identity());

  // Output file
  std::ofstream outfile_gps;
  outfile_gps.open("kitti_gps_0926_0117.txt");
  outfile_gps << std::fixed << std::setprecision(10);
  std::ofstream outfile_imu;
  outfile_imu.open("kitti_imu_0926_0117.txt");
  outfile_imu << std::fixed << std::setprecision(10);
  
  //int stop = 0;
  while (res->next()) {

    //if (stop > 3) break;
    
    // Cargtographer imu to orientation
    // Fetch imu data
    double time = res->getDouble("server_time"); //server_time
    Eigen::Vector3d linear_acceleration(res->getDouble("ax"),
      res->getDouble("ay"), res->getDouble("az"));
    Eigen::Vector3d angular_velocity(res->getDouble("wx") / r_to_d,
      res->getDouble("wy") / r_to_d, res->getDouble("wz") / r_to_d);

    // Add imu data to tracker then get orientation 
    imu_tracker.Advance(time);
    imu_tracker.AddImuAngularVelocityObservation(angular_velocity);
    imu_tracker.Advance(time);
    imu_tracker.AddImuLinearAccelerationObservation(linear_acceleration);
    Eigen::Vector3d orientation = imu_tracker.orientation();
    
    // Output gps data to file 
    outfile_gps << res->getDouble("altitude") << " " 
      << res->getDouble("latitude") << " " 
      << res->getDouble("longitude") << std::endl;
    
    // Output computed orientation to file
    outfile_imu << time << " " << orientation(1) * r_to_d << " " 
      << orientation(0) * r_to_d << " " << orientation(2) * r_to_d << std::endl;

    //stop++;
  }

  //std::cout << z_degree << std::endl;
  outfile_gps.close();
  outfile_imu.close();
  */

  //return EXIT_SUCCESS;
}
