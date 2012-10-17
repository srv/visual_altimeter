#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class VisualAltimeterNode
{
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber point_cloud_sub_;
  ros::Publisher mean_dist_pub_;
  ros::Publisher median_dist_pub_;
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
    nh_private_.param("max_range", max_range_, 3.0);
    nh_private_.param("field_of_view", field_of_view_, 60.0/180.0*M_PI);
    nh_private_.param("min_x", min_x_, -0.2);
    nh_private_.param("max_x", max_x_,  0.2);
    nh_private_.param("min_y", min_y_, -0.2);
    nh_private_.param("max_y", max_y_,  0.2);
    point_cloud_sub_ = nh_.subscribe<PointCloud>("point_cloud", 1, &VisualAltimeterNode::pointCloudCb, this);
    mean_dist_pub_ = nh_private_.advertise<std_msgs::Float32>("mean_distance", 1);
    median_dist_pub_ = nh_private_.advertise<std_msgs::Float32>("median_distance", 1);
    range_pub_ = nh_private_.advertise<sensor_msgs::Range>("altitude", 1);
  }

  void pointCloudCb(const PointCloud::ConstPtr& point_cloud)
  {
    double mean_z, min_z, max_z;
    mean_z = 0.0;
    max_z = 0.0;
    min_z = std::numeric_limits<double>::max();
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
        double z = point.z;
        mean_z += z;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
        count++;
        distances.push_back(z);
      }
    }
    if (count == 0)
    {
      mean_z = -1;
    }
    else
    {
      mean_z /= count;
      median = distances[distances.size()/2];
      ROS_INFO_STREAM("   Z: MIN: " << min_z << "\tMAX: " << max_z << " \tMEAN: " << mean_z << " \tMEDIAN: " << median);
    }
    std_msgs::Float32 dist_msg;
    dist_msg.data = mean_z;
    mean_dist_pub_.publish(dist_msg);
    std_msgs::Float32 median_dist_msg;
    median_dist_msg.data = median;
    median_dist_pub_.publish(median_dist_msg);

    sensor_msgs::Range range_msg;
    range_msg.header = point_cloud->header;
    range_msg.min_range = min_range_;
    range_msg.max_range = max_range_;
    range_msg.field_of_view = field_of_view_;
    range_msg.range = median;

    range_pub_.publish(range_msg);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_altimeter");
  VisualAltimeterNode node;
  ros::spin();
  return 0;
}

