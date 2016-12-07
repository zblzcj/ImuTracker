#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>
/*
  Include directly the different
  headers from cppconn/ and mysql_driver.h + mysql_util.h
  (and mysql_connection.h). This will reduce your build time!
*/
#include "mysql_connection.h"

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include "lib-eigen/Eigen/Geometry"
#include "imu_tracker.h"

# define M_PI           3.14159265358979323846  /* pi */

//using namespace std;

int main(int argc, char * argv[])
{
  //
  /*
  // example 1 ==================================================================
  cout << endl;
  cout << "Running 'SELECT 'Hello World!' » AS _message'..." << endl;

  try {
    sql::Driver *driver;
    sql::Connection *con;
    sql::Statement *stmt;
    sql::ResultSet *res;

    // Create a connection
    driver = get_driver_instance();
    con = driver->connect("tcp://192.168.1.135:3306", "tusimple", "tusimple");
    // Connect to the MySQL test database
    con->setSchema("dope");
    vector<int> tmp;
    tmp.push_back(1);
    tmp.push_back(2);

    stmt = con->createStatement();
    res = stmt->executeQuery("SELECT 'Hello World!' AS _message");
    while (res->next()) {
      cout << "\t... MySQL replies: ";
      // Access column data by alias or column name
      cout << res->getString("_message") << endl;
      cout << "\t... MySQL says it again: ";
      // Access column data by numeric offset, 1 is the first column
      cout << res->getString(1) << endl;
    }
    delete res;
    delete stmt;
    delete con;

  } catch (sql::SQLException &e) {
    cout << "# ERR: SQLException in " << __FILE__;
    //    cout << "(" << __FUNCTION__ << ") on line " »  << __LINE__ << endl;
    cout << "# ERR: " << e.what();
    cout << " (MySQL error code: " << e.getErrorCode();
    cout << ", SQLState: " << e.getSQLState() << " )" << endl;
  }

  cout << endl;
  //*/

  // example 2 ==================================================================
  sql::Driver *driver;
  sql::Connection *con;
  sql::Statement *stmt;

  sql::ResultSet *res;
  sql::PreparedStatement *pstmt;
  driver = get_driver_instance();
  //con = driver->connect("tcp://elder5:3306", "tusimple", "tusimple");
  con = driver->connect("tcp://192.168.1.135:3306", "tusimple", "tusimple");

  stmt = con->createStatement();
  //stmt->execute("USE fkuguan");
  stmt->execute("USE dope");
  //  stmt->execute("DROP TABLE IF EXISTS test");
  //  stmt->execute("CREATE TABLE test(id INT, label CHAR(1))");
  //  stmt->execute("INSERT INTO test(id, label) VALUES (1, 'a')");

  //pstmt = con->prepareStatement("SELECT points from mergetest limit 1;");
  //pstmt = con->prepareStatement("SELECT gps_height from gps_imu limit 10;");
  //pstmt = con->prepareStatement("SELECT server_time FROM mergetest where server_time between  1479879903 and 1479879904 limit 5");

  pstmt = con->prepareStatement("SELECT server_time, angley, anglex, anglez, ay, ax, az, wz, wx, wy, altitude, latitude, longitude FROM gps_imu where server_time between 1479883000 and 1479895000"); // get gps and imu data
  //pstmt = con->prepareStatement("SELECT server_time, points FROM lidar_text where server_time between 1479879903 and 1479879904"); // get points data (string type: x,y,z;)

  //pstmt = con->prepareStatement("SELECT did FROM Data ORDER BY did ASC");
  //pstmt = con->prepareStatement("SELECT tid, path FROM Data ORDER BY did ASC");
  res = pstmt->executeQuery();
  std::cout << "run" << std::endl;
  // Fetch in reverse = descending order!
  //res->afterLast();
  //while (res->previous()) {
  
  res->next();
  const double r_to_d = 180 / M_PI;
  double time = res->getDouble("server_time");
  //Eigen::Vector3d ini_orientation(res->getDouble("anglex") / r_to_d,
    //res->getDouble("angley") / r_to_d, res->getDouble("anglez") / r_to_d);
  //ImuTracker imu_tracker(1e1, time, ImuTracker::toQuaternion(ini_orientation));
  ImuTracker imu_tracker(1e1, time, Eigen::Quaterniond::Identity());

  // Output file
  std::ofstream outfile_gps;
  outfile_gps.open("gps_data_1123.txt");
  outfile_gps << std::fixed << std::setprecision(10);
  std::ofstream outfile_imu;
  outfile_imu.open("imu_data_1123.txt");
  outfile_imu << std::fixed << std::setprecision(10);
  
  //int stop = 0;
  int count_ax = 0, count_ay = 0;
  double ths = 0.25;
  while (res->next()) {

    //if (stop > 3) break;
    
    // Cargtographer imu to orientation
    // Fetch imu data
    double time = res->getDouble("server_time"); //server_time
    Eigen::Vector3d linear_acceleration(res->getDouble("ax"),
      res->getDouble("ay"), res->getDouble("az"));
    Eigen::Vector3d angular_velocity(res->getDouble("wx") / r_to_d,
      res->getDouble("wy") / r_to_d, res->getDouble("wz") / r_to_d);
    
    if (res->getDouble("ax") > ths) 
      count_ax ++;
    if (res->getDouble("ay") > ths) 
      count_ay ++;

    // Add imu data to tracker then get orientation 
    imu_tracker.Advance(time);
    imu_tracker.AddImuAngularVelocityObservation(angular_velocity);
    imu_tracker.Advance(time);
    imu_tracker.AddImuLinearAccelerationObservation(linear_acceleration);
    Eigen::Vector3d orientation = imu_tracker.orientation();
    
    /*
    Eigen::Vector3d gravity = imu_tracker.gravity();
    std::cout << "linear accerleration" << std::endl;
    std::cout << linear_acceleration << std::endl;
    std::cout << "gravity vector" << std::endl; 
    std:: cout << gravity << std::endl;  
    */

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
  std::cout << "ax" << ths << " " << count_ax << std::endl;
  std::cout << "ay" << ths << " " << count_ay << std::endl;

  outfile_gps.close();
  outfile_imu.close();
  delete res;
  delete stmt;
  delete con;

  //return EXIT_SUCCESS;
}
