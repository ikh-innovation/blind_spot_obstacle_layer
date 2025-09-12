#include <blind_spot_obstacle_layer/blind_spot_obstacle_layer.hpp>

#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(blind_spot_obstacle_layer::BlindSpotObstacleLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using costmap_2d::Observation;
using costmap_2d::ObservationBuffer;

namespace blind_spot_obstacle_layer
{

void BlindSpotObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  clear_costmap_srv_ = nh.advertiseService("clear_costmap", &BlindSpotObstacleLayer::clearCostmapCallback, this);
  clear_blind_spot_srv_ =
      nh.advertiseService("clear_blind_spot", &BlindSpotObstacleLayer::clearBlindSpotCallback, this);
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  BlindSpotObstacleLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());
  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);
    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(boost::shared_ptr<ObservationBuffer>(new ObservationBuffer(
        topic, observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height, obstacle_range,
        raytrace_range, *tf_, global_frame_, sensor_frame, transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG("Created an observation buffer for source %s, topic %s, global frame: %s, "
              "expected update rate: %.2f, observation persistence: %.2f",
              source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    if (data_type == "LaserScan")
    {
      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub(
          new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter(
          new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

      if (inf_is_valid)
      {
        filter->registerCallback(
            boost::bind(&BlindSpotObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
      }
      else
      {
        filter->registerCallback(
            boost::bind(&BlindSpotObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud>> sub(
          new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
        ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud>> filter(
          new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
      filter->registerCallback(
          boost::bind(&BlindSpotObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else
    {
      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(
          new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
        ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter(
          new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
      filter->registerCallback(
          boost::bind(&BlindSpotObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector<std::string> target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  clear_next_recovery_polygon_ = false;
  clear_next_recovery_polygon_index_ = -1;
  std::string blind_spot_obstacle_layer_topic, blind_spot_frame, recovery_clearance_polygons_frame,
      recovery_clearance_polygons_topic;
  nh.param("blind_spot_polygon_marker_topic", blind_spot_obstacle_layer_topic, std::string("blind_spot_polygon"));
  nh.param("recovery_clearance_polygons_topic", recovery_clearance_polygons_topic,
           std::string("recovery_clearance_polygons"));
  nh.param("blind_spot_frame", blind_spot_frame, std::string("base_link"));
  nh.param("recovery_clearance_polygons_frame", recovery_clearance_polygons_frame, std::string("base_link"));
  blind_spot_marker_pub_ = nh.advertise<visualization_msgs::Marker>(blind_spot_obstacle_layer_topic, 10);

  // TODO: Enhancement1: add a combo enum on dynamic reconfigure for some predefined shapes along with their values
  // (as an array of doubles)

  // TODO: Enhancement2: add a polygon per observation source instead of the general one below
  blind_spot_polygon_marker_.header.frame_id = blind_spot_frame;
  blind_spot_polygon_marker_.header.stamp = ros::Time::now();
  blind_spot_polygon_marker_.ns = "blind_spot_obstacle_layer_polygon_" + blind_spot_frame;
  blind_spot_polygon_marker_.id = 0;
  blind_spot_polygon_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  blind_spot_polygon_marker_.action = visualization_msgs::Marker::ADD;
  blind_spot_polygon_marker_.pose.orientation.w = 1.0;
  blind_spot_polygon_marker_.scale.x = 0.1;
  blind_spot_polygon_marker_.color.r = 1.0f;
  blind_spot_polygon_marker_.color.g = 0.0f;
  blind_spot_polygon_marker_.color.b = 0.0f;
  blind_spot_polygon_marker_.color.a = 1.0;

  XmlRpc::XmlRpcValue polygon_xml;
  if (nh.hasParam("blind_spot_polygon"))
  {
    nh.getParam("blind_spot_polygon", polygon_xml);
    getPolygonFromYaml(polygon_xml, blind_spot_polygon_marker_);
  }

  if (blind_spot_polygon_marker_.points.empty())
  {
    ROS_WARN("No blind_spot_polygon was set. Falling back to default...");
    for (auto p : std::vector<std::vector<double>>{ { 1, 1 }, { 1, -1 }, { -1, -1 }, { -1, 1 } })
    {
      geometry_msgs::Point point;
      point.x = p[0];
      point.y = p[1];
      blind_spot_polygon_marker_.points.push_back(point);
    }
  }
  if (blind_spot_polygon_marker_.points.front().x != blind_spot_polygon_marker_.points.back().x or
      blind_spot_polygon_marker_.points.front().y != blind_spot_polygon_marker_.points.back().y)
  {
    blind_spot_polygon_marker_.points.push_back(blind_spot_polygon_marker_.points.front());
  }
  for (auto p : blind_spot_polygon_marker_.points)
  {
    transformed_blind_spot_polygon_.push_back(p);
  }

  if (nh.hasParam("recovery_clearance_polygons"))
  {
    nh.getParam("recovery_clearance_polygons", polygon_xml);
    getArrayOfPolygonsFromYaml(polygon_xml, recovery_clearance_polygons_frame, recovery_clearance_polygons_);

    for (auto polygon : recovery_clearance_polygons_)
    {
      std::vector<geometry_msgs::Point> v;
      for (auto p : polygon.points)
      {
        v.push_back(p);
      }
      transformed_recovery_clearance_polygons_.push_back(v);
    }

    for (int i = 0; i < recovery_clearance_polygons_.size(); i++)
    {
      ros::Publisher p =
          nh.advertise<visualization_msgs::Marker>(recovery_clearance_polygons_topic + std::to_string(i), 10);
      recovery_clearance_polygons_pubs_.push_back(p);
      auto clear_recovery_polygon_lambda = [this, i](std_srvs::Trigger::Request& req,
                                                     std_srvs::Trigger::Response& res) -> bool {
        return this->clearRecoveryPolygonCallback(i, req, res);
      };
      boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> crpl =
          clear_recovery_polygon_lambda;
      ros::ServiceServer s = nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "clear_recovery_polygon" + std::to_string(i), crpl);
      clear_next_recovery_polygon_srvs_.push_back(s);
    }
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void BlindSpotObstacleLayer::getArrayOfPolygonsFromYaml(const XmlRpc::XmlRpcValue& polygons_xml,
                                                        const std::string& markers_frame,
                                                        std::vector<visualization_msgs::Marker>& polygons)
{
  if (polygons_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < polygons_xml.size(); ++i)
    {
      if (polygons_xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = markers_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "recovery_clearance_polygons_" + std::to_string(i);
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        auto c = i % 6;
        if (c == 0)
        {
          marker.color.r = 1.0;
          marker.color.g = 1.0;
        }
        else if (c == 1)
        {
          marker.color.r = 1.0;
          marker.color.b = 1.0;
        }
        else if (c == 2)
        {
          marker.color.g = 1.0;
          marker.color.b = 1.0;
        }
        else if (c == 3)
        {
          marker.color.b = 1.0;
        }
        else if (c == 4)
        {
          marker.color.g = 1.0;
        }
        else if (c == 5)
        {
          marker.color.r = 1.0;
        }
        else
        {
          marker.color.r = 1.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0;
        }
        getPolygonFromYaml(polygons_xml[i], marker);
        polygons.push_back(marker);
      }
    }
  }
}

bool BlindSpotObstacleLayer::getPolygonFromYaml(const XmlRpc::XmlRpcValue& polygon_xml,
                                                visualization_msgs::Marker& vis_marker)
{
  try
  {
    if (polygon_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < polygon_xml.size(); ++i)
      {
        // NOTE: The blind spot polygon is a 2d array of doubles (or ints)
        // I am using invalid_type to first check if the next element in
        // the xml value is an array and then I re-use it to check the
        // types of the elements of that array
        std::vector<double> point;
        if (polygon_xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          for (int j = 0; j < polygon_xml[i].size(); ++j)
          {
            if (polygon_xml[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            {
              point.push_back(static_cast<double>(polygon_xml[i][j]));
            }
            else if (polygon_xml[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
            {
              point.push_back(static_cast<int>(polygon_xml[i][j]));
            }
            else
            {
              ROS_WARN("Unexpected value while reading the blind_spot_polygon parameter. Supported values: 2d array "
                       "of doubles or integers.");
              vis_marker.points.clear();
              return false;
            }
          }
          if (point.size() == 2)
          {
            geometry_msgs::Point p;
            p.x = point[0];
            p.y = point[1];
            vis_marker.points.push_back(p);
          }
        }
      }
      if (not vis_marker.points.empty() and (vis_marker.points.front().x != vis_marker.points.back().x or
                                             vis_marker.points.front().y != vis_marker.points.back().y))
      {
        vis_marker.points.push_back(vis_marker.points.front());
      }
    }
  }
  catch (XmlRpc::XmlRpcException e)
  {
    ROS_WARN("Error while trying to read the polygon:: %s", e.getMessage().c_str());
  }
  return true;
}

void BlindSpotObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<blind_spot_obstacle_layer::BlindSpotObstacleLayerConfig>(nh);
  dynamic_reconfigure::Server<blind_spot_obstacle_layer::BlindSpotObstacleLayerConfig>::CallbackType cb =
      boost::bind(&BlindSpotObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

BlindSpotObstacleLayer::~BlindSpotObstacleLayer()
{
  if (dsrv_)
    delete dsrv_;
}
void BlindSpotObstacleLayer::reconfigureCB(blind_spot_obstacle_layer::BlindSpotObstacleLayerConfig& config,
                                           uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  min_obstacle_height_ = config.min_obstacle_height;
  combination_method_ = config.combination_method;
  blind_spot_combination_method_ = config.blind_spot_combination_method;
  publish_blind_spot_marker_ = config.publish_blind_spot_marker;
  publish_recovery_clearance_polygons_ = config.publish_recovery_clearance_polygons;
  blind_spot_enabled_ = config.blind_spot_enabled;
}

void BlindSpotObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(*message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void BlindSpotObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                                       const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[i];
    if (!std::isfinite(range) && range > 0)
    {
      message.ranges[i] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void BlindSpotObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  sensor_msgs::PointCloud2 cloud2;

  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void BlindSpotObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                 const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

bool BlindSpotObstacleLayer::saveOldCostmap(std::vector<std::pair<unsigned int, uint8_t>>& old_costmap, double robot_x,
                                            double robot_y, double robot_yaw)
{
  std::vector<costmap_2d::MapLocation> map_polygon;
  for (unsigned int i = 0; i < transformed_blind_spot_polygon_.size(); i++)
  {
    costmap_2d::MapLocation loc;
    if (!worldToMap(transformed_blind_spot_polygon_[i].x, transformed_blind_spot_polygon_[i].y, loc.x, loc.y))
    {
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<costmap_2d::MapLocation> polygon_cells;

  convexFillCells(map_polygon, polygon_cells);

  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    old_costmap.push_back(std::make_pair(index, costmap_[index]));
  }
  return true;
}

bool BlindSpotObstacleLayer::restoreBasedOnOldCostmap(std::vector<std::pair<unsigned int, uint8_t>>& old_costmap)
{
  if (blind_spot_combination_method_ > 2)
  {
    return false;
  }
  if (blind_spot_combination_method_ == 0)
  {
    for (auto pair : old_costmap)
    {
      costmap_[pair.first] = pair.second;
    }
  }
  else if (blind_spot_combination_method_ == 1)
  {
    for (auto pair : old_costmap)
    {
      costmap_[pair.first] = std::max(costmap_[pair.first], pair.second);
    }
  }
  else if (blind_spot_combination_method_ == 2)
  {
    for (auto pair : old_costmap)
    {
      costmap_[pair.first] = std::min(costmap_[pair.first], pair.second);
    }
  }
  return true;
}

void BlindSpotObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  useExtraBounds(min_x, min_y, max_x, max_y);
  std::vector<std::pair<unsigned int, uint8_t>> old_costmap;
  if (blind_spot_enabled_)
  {
    costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, blind_spot_polygon_marker_.points,
                                   transformed_blind_spot_polygon_);
    saveOldCostmap(old_costmap, robot_x, robot_y, robot_yaw);
  }

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        continue;
      }
      if (pz < min_obstacle_height_)
      {
        ROS_DEBUG("The point is too low");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }
  if (skip_next_blind_spot_)
  {
    old_costmap.clear();
    skip_next_blind_spot_ = false;
  }
  if (blind_spot_enabled_)
  {
    restoreBasedOnOldCostmap(old_costmap);
  }

  for (int i = 0; i < recovery_clearance_polygons_.size(); i++)
  {
    transformed_recovery_clearance_polygons_[i].clear();
    std::vector<geometry_msgs::Point> v;
    costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, recovery_clearance_polygons_[i].points, v);
    for (auto p : v)
    {
      transformed_recovery_clearance_polygons_[i].push_back(p);
    }
  }

  if (publish_blind_spot_marker_)
  {
    blind_spot_polygon_marker_.header.stamp = ros::Time::now();
    blind_spot_marker_pub_.publish(blind_spot_polygon_marker_);
  }
  if (publish_recovery_clearance_polygons_)
  {
    for (int i = 0; i < recovery_clearance_polygons_.size(); i++)
    {
      recovery_clearance_polygons_[i].header.stamp = ros::Time::now();
      recovery_clearance_polygons_pubs_[i].publish(recovery_clearance_polygons_[i]);
    }
  }
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void BlindSpotObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                             double* min_y, double* max_x, double* max_y)
{
  if (!footprint_clearing_enabled_)
    return;
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, costmap_2d::Layer::getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
  {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
  }
}

void BlindSpotObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  if (clear_next_costmap_)
  {
    resetMaps();
    clear_next_costmap_ = false;
  }
  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }
  if (clear_next_recovery_polygon_ and clear_next_recovery_polygon_index_ >= 0)
  {
    setConvexPolygonCost(transformed_recovery_clearance_polygons_[clear_next_recovery_polygon_index_],
                         costmap_2d::FREE_SPACE);

    clear_next_recovery_polygon_ = false;
    clear_next_recovery_polygon_index_ = -1;
  }
  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void BlindSpotObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void BlindSpotObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool BlindSpotObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(), static_marking_observations_.begin(),
                              static_marking_observations_.end());
  return current;
}

bool BlindSpotObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(), static_clearing_observations_.begin(),
                               static_clearing_observations_.end());
  return current;
}

// Function to check if a point lies on a given line segment

void BlindSpotObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                               double* max_x, double* max_y)
{
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::PointCloud2& cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;
  if (!worldToMap(ox, oy, x0, y0))
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;
  double map_end_x = origin_x + size_x_ * resolution_;
  double map_end_y = origin_y + size_y_ * resolution_;

  touch(ox, oy, min_x, min_y, max_x, max_y);

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    double wx = *iter_x;
    double wy = *iter_y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    double a = wx - ox;
    double b = wy - oy;

    // the minimum value to raytrace from is the origin
    if (wx < origin_x)
    {
      double t = (origin_x - ox) / a;
      wx = origin_x;
      wy = oy + b * t;
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    unsigned int x1, y1;

    // check for legality just in case
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void BlindSpotObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void BlindSpotObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void BlindSpotObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                                  double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx - ox, dy = wy - oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void BlindSpotObstacleLayer::reset()
{
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

bool BlindSpotObstacleLayer::clearBlindSpotCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  skip_next_blind_spot_ = true;
  res.success = skip_next_blind_spot_;
  return true;
}

bool BlindSpotObstacleLayer::clearCostmapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  clear_next_costmap_ = true;
  res.success = clear_next_costmap_;
  return true;
}

bool BlindSpotObstacleLayer::clearRecoveryPolygonCallback(const int polygon_index, std_srvs::Trigger::Request& req,
                                                          std_srvs::Trigger::Response& res)
{
  clear_next_recovery_polygon_ = true;
  clear_next_recovery_polygon_index_ = polygon_index;
  res.success = clear_next_recovery_polygon_index_ != -1;
  return true;
}

}  // namespace blind_spot_obstacle_layer
