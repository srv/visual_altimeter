#include <ros/ros.h>

#include <numeric>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class VisualAltimeterNode
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher range_pub_;

  double min_range_;
  double max_range_;
  double field_of_view_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;

  vector<double> filter_;

public:
  VisualAltimeterNode() : nh_private_("~")
  {
    nh_private_.param("min_range", min_range_, 0.5);
    nh_private_.param("max_range", max_range_, 6.0);
    nh_private_.param("field_of_view", field_of_view_, 60.0/180.0*M_PI);
    nh_private_.param("min_x", min_x_, -0.3);
    nh_private_.param("max_x", max_x_,  0.3);
    nh_private_.param("min_y", min_y_, -0.3);
    nh_private_.param("max_y", max_y_,  0.3);
    point_cloud_sub_ = nh_.subscribe<PointCloud>("point_cloud", 1, &VisualAltimeterNode::pointCloudCb, this);
    range_pub_ = nh_private_.advertise<sensor_msgs::Range>("altitude", 1);
  }

  void pointCloudCb(const PointCloud::ConstPtr& point_cloud)
  {
    if(range_pub_.getNumSubscribers() == 0) return;

    vector<double> z_dist;
    for (size_t i = 0; i < point_cloud->points.size(); ++i)
    {
      const pcl::PointXYZ& point = point_cloud->points[i];
      if (point.x >= min_x_ && point.x <= max_x_ &&
          point.y >= min_y_ && point.y <= max_y_ &&
          !isnan(point.z))
      {
        z_dist.push_back(point.z);
      }
    }

    // Sanity check and filtering
    double altitude = -1;

    if (z_dist.size() > 300)
    {
      double max_val = *max_element(z_dist.begin(), z_dist.end());
      double min_val = *min_element(z_dist.begin(), z_dist.end());
      if (fabs(max_val - min_val) < 1.0)
      {
        double mean = accumulate(z_dist.begin(), z_dist.end(), 0.0) / z_dist.size();

        // Filter (5 samples)
        if (filter_.size() < 5)
          filter_.push_back(mean);
        else
        {
          rotate(filter_.begin(), filter_.begin() + 1, filter_.end());
          filter_[4] = mean;
        }
        altitude = accumulate(filter_.begin(), filter_.end(), 0.0) / filter_.size();
      }
    }

    // Publish if valid altitude
    if (altitude < max_range_ && altitude > min_range_)
    {
      sensor_msgs::Range range_msg;
      range_msg.header = pcl_conversions::fromPCL(point_cloud->header);
      range_msg.min_range = min_range_;
      range_msg.max_range = max_range_;
      range_msg.field_of_view = field_of_view_;
      range_msg.range = altitude;
      range_pub_.publish(range_msg);
    } else {
      ROS_INFO_THROTTLE(10, "[VisualAltimeter]: Invalid altitude");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_altimeter");
  VisualAltimeterNode node;
  ros::spin();
  return 0;
}

