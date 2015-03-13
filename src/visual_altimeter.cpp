#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

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

public:
  VisualAltimeterNode() : nh_private_("~")
  {
    nh_private_.param("min_range", min_range_, 0.2);
    nh_private_.param("max_range", max_range_, 4.0);
    nh_private_.param("field_of_view", field_of_view_, 60.0/180.0*M_PI);
    nh_private_.param("min_x", min_x_, -0.2);
    nh_private_.param("max_x", max_x_,  0.2);
    nh_private_.param("min_y", min_y_, -0.2);
    nh_private_.param("max_y", max_y_,  0.2);
    point_cloud_sub_ = nh_.subscribe<PointCloud>("point_cloud", 1, &VisualAltimeterNode::pointCloudCb, this);
    range_pub_ = nh_private_.advertise<sensor_msgs::Range>("altitude", 1);
  }

  void pointCloudCb(const PointCloud::ConstPtr& point_cloud)
  {
    int count = 0;
    std::vector<double> distances;
    double median = -1;
    for (size_t i = 0; i < point_cloud->points.size(); ++i)
    {
      const pcl::PointXYZ& point = point_cloud->points[i];
      if (point.x >= min_x_ && point.x <= max_x_ &&
          point.y >= min_y_ && point.y <= max_y_ &&
          !std::isnan(point.z))
      {
        count++;
        distances.push_back(point.z);
      }
    }

    // Sanity check
    if (count < 300) // Minimum number of samples
    {
      median = -1;
    }
    else
    {
      median = distances[distances.size()/2];
    }

    // Publish if valid altitude
    if (median < max_range_ && median > min_range_)
    {
      sensor_msgs::Range range_msg;
      range_msg.header = pcl_conversions::fromPCL(point_cloud->header);
      range_msg.min_range = min_range_;
      range_msg.max_range = max_range_;
      range_msg.field_of_view = field_of_view_;
      range_msg.range = median;
      range_pub_.publish(range_msg);
    } else {
      ROS_INFO_THROTTLE(10, "[VisualAltimeter]: Invalid altitude. Number of samples: %d", count);
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

