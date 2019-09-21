
#include <string>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "cartographer/common/time.h"
#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/probability_grid.h"
#include "cartographer/mapping/probability_grid_range_data_inserter_2d.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/transform.h"
#include "cartographer/mapping/probability_values.h"
#include <boost/thread.hpp>
#include "cartographer/location/laser_location.h"
#include "ctime"
#include "drives/pepperl_fuchs_r2000/include/r2000_driver.h"
#include <boost/asio.hpp>
#include <boost/timer.hpp>


using namespace cartographer;
using namespace mapping;
using namespace common;
using namespace sensor;
using namespace cartographer::mapping::scan_matching;

#undef main 
using namespace std;
namespace boost
{
	void throw_exception(std::exception const & e) // user defined
	{
		return;
	}
}
common::Time FromNow() {

//	SYSTEMTIME sys;
//	GetLocalTime(&sys);

    return ::cartographer::common::FromUniversal(
        (1 +
            ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
        10000000ll +
        (1 + 50) / 100);  // + 50 to get the rounding correct.

}

int frequency = 35;
 float kMinScore = 0.1f;
 cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
 CreateFastCorrelativeScanMatcherTestOptions2D(
     const int branch_and_bound_depth) {
   auto parameter_dictionary =
       common::MakeDictionary(R"text(
       return {
          linear_search_window = 3.,
          angular_search_window = 1.,
          branch_and_bound_depth = )text" +
                              std::to_string(branch_and_bound_depth) + "}");
   return CreateFastCorrelativeScanMatcherOptions2D(parameter_dictionary.get());
 }
std::unique_ptr<cartographer::location::Laserlocation>  location_;

uint8 ProbabilityToColor(float probability_from_grid) {
  const float probability = probability_from_grid;
  return ::cartographer::common::RoundToInt(
      255 * ((probability - mapping::kMinProbability) /
             (mapping::kMaxProbability - mapping::kMinProbability)));
}
class nodeslam
{
public:
    nodeslam();
    ~nodeslam();
  
    void ScanMathing(sensor::TimedPointCloudData* pcl);
	void ReceiveData(double useless);
	void Start();
	void ReceiveData_temp(double a);
	
private:
    
      transform::Rigid2d pose_estimate;
      boost::thread* transform_thread_;
	  boost::thread* transform_thread_temp;
 
      boost::mutex map_to_odom_mutex_;
      transform::Rigid2d pose_estimate_temp1;
	  pepperl_fuchs::R2000Driver r2000_drivers;
       bool first_in;
       int count;
    //   std::unique_ptr<Server> server;
};
nodeslam::nodeslam()
{
	
}
nodeslam::~nodeslam()
{
    if(transform_thread_)
     {
       transform_thread_->join();
       delete transform_thread_;
     }
}

void nodeslam::ScanMathing(sensor::TimedPointCloudData* point_cloud)
{
         location_->ProcessLaserdata(*point_cloud);
         transform::Rigid3d pose_estimate =
         transform::Embed3D((location_->current_pose));      
         sensor::TimedPointCloud rotated_point= sensor::TransformTimedPointCloud(point_cloud->ranges, pose_estimate.cast<float>());
}

void nodeslam::ReceiveData(double useless)
{
	while (true)
	{
		boost::this_thread::sleep(boost::posix_time::microseconds((int)(1000 / frequency)));
		int scans_available = r2000_drivers.getFullScansAvailable();
		pepperl_fuchs::ScanData  scandata = r2000_drivers.getFullScan();
		if (scandata.amplitude_data.empty() || scandata.distance_data.empty())
			continue;
		sensor::TimedPointCloudData point_cloud;
		point_cloud.time = FromNow();
		double angle_min = -M_PI;
		double angle_max = M_PI;
		double angle_increment = 2 * M_PI / float(scandata.distance_data.size());
		for (int i = 0; i < scandata.distance_data.size(); i++)
		{
			double angle = angle_min + angle_increment * i;
			const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
			float dis = (scandata.distance_data[i] / 1000.0f);
			Eigen::Vector3f point(rotation * (dis* Eigen::Vector3f::UnitX()));
			float x = (float)(dis * cos(angle));
			float y = (float)(dis* sin(angle));
			Eigen::Vector3f point_1(x, y, 0.f);
			point_cloud.ranges.push_back({ point });
		}
		boost::timer tim;
		ScanMathing(&point_cloud);
		std::cout << tim.elapsed() << std::endl;
	}
	
}
void nodeslam::ReceiveData_temp(double a)
{
	while (true)
	{
		boost::this_thread::sleep(boost::posix_time::microseconds((int)(300)));
		int scans_available = r2000_drivers.getFullScansAvailable();
		pepperl_fuchs::ScanData  scandata = r2000_drivers.getFullScan();
		if (scandata.amplitude_data.empty() || scandata.distance_data.empty())
		{

		}
		else
		{
			std::cout << "read data" << std::endl;
		}
	}
}
void nodeslam::Start()
{
	r2000_drivers.connect("10.0.10.9", 80);
	r2000_drivers.setScanFrequency(frequency);
	r2000_drivers.setSamplesPerScan(3600);

	auto params = r2000_drivers.getParameters();
	std::cout << "Current scanner settings:" << std::endl;
	std::cout << "============================================================" << std::endl;
	for (const auto& p : params)
		std::cout << p.first << " : " << p.second << std::endl;
	std::cout << "============================================================" << std::endl;
	if (r2000_drivers.startCapturingTCP())
	{
		transform_thread_ = new boost::thread(boost::bind(&nodeslam::ReceiveData, this, 0.001));
		transform_thread_temp= new boost::thread(boost::bind(&nodeslam::ReceiveData_temp, this, 0.001));
	}
	else
	{
		std::cout << "FAILED!" << std::endl;
	}
}



int main(int argc, char *argv[])
{

     std::string dir="";
     std::string name="option.lua";
     location_=absl::make_unique<cartographer::location::Laserlocation>(dir,name);	
	 nodeslam slam;	
	 slam.Start();
     std::cout<<"waiting data"<<std::endl; 

	return 0;
}
