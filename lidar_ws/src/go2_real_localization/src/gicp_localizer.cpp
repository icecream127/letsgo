#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <go2_obstacle_interfaces/msg/localization_evidence.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <random>
#include <iomanip>
#include <future>

using CloudMsg = sensor_msgs::msg::PointCloud2;
using Odom = nav_msgs::msg::Odometry;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Matrix = Eigen::Matrix4f;
using Policy = message_filters::sync_policies::ExactTime<CloudMsg, Odom>;

Matrix poseMatrix(const geometry_msgs::msg::Pose & p) {
  Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) throw std::runtime_error("Invalid quaternion");
  Matrix t = Matrix::Identity();
  t.block<3,3>(0,0) = q.normalized().toRotationMatrix();
  t.block<3,1>(0,3) = Eigen::Vector3f(p.position.x,p.position.y,p.position.z);
  if (!t.allFinite()) throw std::runtime_error("Non-finite pose");
  return t;
}
geometry_msgs::msg::Pose toPose(const Matrix & t) {
  geometry_msgs::msg::Pose p;
  p.position.x=t(0,3); p.position.y=t(1,3); p.position.z=t(2,3);
  Eigen::Quaternionf q(t.block<3,3>(0,0)); q.normalize();
  p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
  return p;
}

class Localizer : public rclcpp::Node {
public:
  Localizer() : Node("gicp_localizer") {
    // Process identity prevents late DDS samples from a prior localizer instance
    // being accepted as the current source after a restart.
    std::random_device random;
    std::ostringstream identity;
    identity << std::hex << std::setfill('0');
    for (int i=0;i<4;++i) identity << std::setw(8) << random();
    source_id_=identity.str();
    map_frame_=declare_parameter("map_frame", "map");
    odom_frame_=declare_parameter("odom_frame", "odom");
    body_frame_=declare_parameter("body_frame", "livox_frame");
    if (map_frame_==odom_frame_ || body_frame_==odom_frame_ || body_frame_==map_frame_)
      throw std::runtime_error("Frames must be distinct");
    voxel_=declare_parameter("voxel_size",0.15);
    radius_=declare_parameter("local_radius",25.0);
    period_=declare_parameter("registration_period",1.0);
    max_age_=declare_parameter("max_input_age",2.0);
    ttl_=declare_parameter("transform_timeout",3.0);
    iterations_=declare_parameter("max_iterations",40);
    threads_=declare_parameter("num_threads",4);
    distance_=declare_parameter("correspondence_distance",1.0);
    inlier_distance_=declare_parameter("inlier_distance",0.30);
    ratio_=declare_parameter("min_inlier_ratio",0.60);
    rmse_=declare_parameter("max_rmse",0.20);
    min_points_=declare_parameter("min_points",100);
    correction_t_=declare_parameter("max_correction_translation",0.5);
    correction_r_=declare_parameter("max_correction_rotation",0.35);
    initial_t_=declare_parameter("initial_max_translation",2.0);
    initial_r_=declare_parameter("initial_max_rotation",0.8);
    if (voxel_<=0 || radius_<=0 || period_<=0 || max_age_<=0 || ttl_<=0 ||
        iterations_<1 || threads_<1 || min_points_<20 || distance_<=0 || inlier_distance_<=0 ||
        ratio_<=0 || ratio_>1 || rmse_<=0 || correction_t_<=0 || correction_r_<=0 ||
        initial_t_<=0 || initial_r_<=0) throw std::runtime_error("Invalid GICP parameters");
    auto file=declare_parameter<std::string>("map_path", "");
    Cloud::Ptr raw(new Cloud);
    if (file.empty() || pcl::io::loadPCDFile(file,*raw)<0) throw std::runtime_error("Cannot load map_path: "+file);
    map_=filtered(raw);
    if (map_->size()<static_cast<size_t>(min_points_)) throw std::runtime_error("Map too small");
    registration_.setRegistrationType("GICP");
    registration_.setNumThreads(threads_);
    registration_.setMaximumIterations(iterations_);
    registration_.setMaxCorrespondenceDistance(distance_);
    registration_.setTransformationEpsilon(5e-4);
    registration_.setRotationEpsilon(2e-3);
    registration_.setCorrespondenceRandomness(20);
    map_pub_=create_publisher<CloudMsg>("gicp/map",rclcpp::QoS(1).transient_local());
    aligned_pub_=create_publisher<CloudMsg>("gicp/aligned_cloud",1);
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("gicp/pose",1);
    status_pub_=create_publisher<std_msgs::msg::String>("gicp/status",rclcpp::QoS(1).transient_local());
    valid_pub_=create_publisher<std_msgs::msg::Bool>("gicp/valid",rclcpp::QoS(1).transient_local());
    evidence_pub_=create_publisher<go2_obstacle_interfaces::msg::LocalizationEvidence>("gicp/evidence",10);
    input_status_pub_=create_publisher<std_msgs::msg::String>("gicp/input_status",10);
    broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    CloudMsg map_msg; pcl::toROSMsg(*map_,map_msg);
    map_msg.header.frame_id=map_frame_; map_msg.header.stamp=now(); map_pub_->publish(map_msg);
    pending_=declare_parameter("initial_pose_enabled",false);
    auto initial=declare_parameter<std::vector<double>>("initial_pose",{0,0,0,0,0,0});
    if (initial.size()!=6 || !std::all_of(initial.begin(),initial.end(),[](double x){return std::isfinite(x);}))
      throw std::runtime_error("initial_pose must contain finite x,y,z,roll,pitch,yaw");
    seed_=Matrix::Identity();
    seed_.block<3,1>(0,3)=Eigen::Vector3f(initial[0],initial[1],initial[2]);
    seed_.block<3,3>(0,0)=(Eigen::AngleAxisf(initial[5],Eigen::Vector3f::UnitZ())*
      Eigen::AngleAxisf(initial[4],Eigen::Vector3f::UnitY())*
      Eigen::AngleAxisf(initial[3],Eigen::Vector3f::UnitX())).toRotationMatrix();
    initial_sub_=create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
        if(m->header.frame_id!=map_frame_) {RCLCPP_WARN(get_logger(),"Initial pose must be in map frame"); return;}
        try {seed_=poseMatrix(m->pose.pose);} catch(const std::exception &e){RCLCPP_WARN(get_logger(),"%s",e.what());return;}
        ++epoch_; accepted_stamp_=0; last_processed_=0;
        pending_=true; initialized_=false; valid_=false; report("INITIAL_POSE_RECEIVED");
      });
    // Match FAST-LIO's registered-body Reliable publisher. Keep the complete
    // selected profile explicit; alternative publishers may select BestEffort.
    // Input diagnostics below distinguish raw arrival from ExactTime pairing;
    // this QoS choice alone is not evidence that a discovered stream is usable.
    const auto cloud_reliability=declare_parameter<std::string>("cloud_reliability", "reliable");
    if(cloud_reliability!="reliable" && cloud_reliability!="best_effort")
      throw std::runtime_error("cloud_reliability must be reliable or best_effort");
    auto cloud_qos=cloud_reliability=="reliable" ? rmw_qos_profile_default : rmw_qos_profile_sensor_data;
    const auto cloud_depth=declare_parameter<int>("cloud_qos_depth",static_cast<int>(cloud_qos.depth));
    if(cloud_depth<1 || cloud_depth>100) throw std::runtime_error("cloud_qos_depth must be in [1,100]");
    cloud_qos.depth=static_cast<size_t>(cloud_depth);
    cloud_sub_.subscribe(this,declare_parameter("cloud_topic", "/cloud_registered_body"),cloud_qos);
    odom_sub_.subscribe(this,declare_parameter("odom_topic", "/Odometry"),rmw_qos_profile_sensor_data);
    // Observe raw arrivals without modifying/filtering messages passed to
    // ExactTime. This separates absent clouds, absent odometry, and unpaired
    // timestamps from registration-quality rejection in field diagnostics.
    cloud_input_connection_=cloud_sub_.registerCallback([this](CloudMsg::ConstSharedPtr m){
      ++cloud_input_count_;
      cloud_input_stamp_=static_cast<int64_t>(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;
    });
    odom_input_connection_=odom_sub_.registerCallback([this](Odom::ConstSharedPtr m){
      ++odom_input_count_;
      odom_input_stamp_=static_cast<int64_t>(m->header.stamp.sec)*1000000000LL+m->header.stamp.nanosec;
    });
    sync_=std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20),cloud_sub_,odom_sub_);
    sync_->registerCallback(std::bind(&Localizer::onPair,this,std::placeholders::_1,std::placeholders::_2));
    registration_timer_=create_wall_timer(std::chrono::duration<double>(period_),[this]{match();});
    result_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{finishMatch();});
    tf_timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{broadcast();});
    report("WAITING_FOR_SYNCHRONIZED_INPUT_AND_INITIALPOSE");
    RCLCPP_INFO(get_logger(),"Loaded %zu map points; using small_gicp::RegistrationPCL (%d threads)",map_->size(),threads_);
  }

  ~Localizer() override {
    if (match_future_.valid()) match_future_.wait();
  }

private:
  struct MatchResult {
    std::string rejection;
    Matrix pose = Matrix::Identity();
    CloudMsg aligned;
    double overlap = 0.0;
    double error = 0.0;
    double translation = 0.0;
    double rotation = 0.0;
    double match_ms = 0.0;
    bool target_reused = false;
  };

  void onPair(const CloudMsg::ConstSharedPtr &c,const Odom::ConstSharedPtr &o) {
    ++paired_input_count_;
    if(c->header.frame_id!=body_frame_ || o->header.frame_id!=odom_frame_ || o->child_frame_id!=body_frame_){
      ++epoch_;
      valid_=false; cloud_.reset(); odom_.reset(); report("FRAME_MISMATCH");return;
    }
    const auto stamp=rclcpp::Time(c->header.stamp).nanoseconds();
    if(last_input_ && stamp<last_input_){
      ++epoch_; accepted_stamp_=0; last_processed_=0;
      pending_=false;initialized_=false;valid_=false;report("TIME_RESET_NEEDS_INITIALPOSE");
    }
    last_input_=stamp; cloud_=c; odom_=o;
  }
  Cloud::Ptr filtered(const Cloud::Ptr & input) {
    Cloud::Ptr clean(new Cloud), out(new Cloud);
    for(const auto &p:*input) if(pcl::isFinite(p)) clean->push_back(p);
    pcl::VoxelGrid<pcl::PointXYZ> vg; vg.setLeafSize(voxel_,voxel_,voxel_);
    vg.setInputCloud(clean); vg.filter(*out); return out;
  }
  void report(const std::string & s) {
    std::ostringstream inputs;
    inputs << "{\"cloud_count\":" << cloud_input_count_ << ",\"odom_count\":" << odom_input_count_
      << ",\"pair_count\":" << paired_input_count_ << ",\"cloud_stamp_ns\":" << cloud_input_stamp_
      << ",\"odom_stamp_ns\":" << odom_input_stamp_ << "}";
    std_msgs::msg::String input_status;input_status.data=inputs.str();input_status_pub_->publish(input_status);
    std_msgs::msg::String msg;msg.data=s;status_pub_->publish(msg);
    std_msgs::msg::Bool b;b.data=valid_;valid_pub_->publish(b);
    go2_obstacle_interfaces::msg::LocalizationEvidence e;
    e.header.frame_id=map_frame_;
    // Accepted evidence retains the sensor measurement time, never a TF refresh
    // or timer time. Rejection stamps describe an event, not a new measurement.
    if(valid_) e.header.stamp=rclcpp::Time(accepted_stamp_);
    else e.header.stamp=now();
    e.source_id=source_id_; e.epoch=epoch_; e.sequence=++sequence_;
    e.accepted=valid_; e.body_frame=body_frame_; e.reason=s;
    e.covariance_known=false; // GICP residual is not calibrated pose covariance.
    e.pose.pose=valid_ ? accepted_pose_ : geometry_msgs::msg::Pose();
    if(!valid_) e.pose.pose.orientation.w=1.0;
    evidence_pub_->publish(e);
  }
  // Only the worker touches the registration object and target caches.
  MatchResult registerCloud(const CloudMsg & cloud, const Matrix & guess) {
    MatchResult result;
    const auto start=std::chrono::steady_clock::now();
    Cloud::Ptr raw(new Cloud);
    pcl::fromROSMsg(cloud,*raw);
    auto source=filtered(raw);
    std::vector<std::size_t> indices;
    indices.reserve(map_->size());
    const Eigen::Vector3f center=guess.block<3,1>(0,3);
    const double squared_radius=radius_*radius_;
    for(std::size_t i=0;i<map_->size();++i) {
      if(((*map_)[i].getVector3fMap()-center).squaredNorm()<squared_radius) indices.push_back(i);
    }
    if(source->size()<static_cast<std::size_t>(min_points_) || indices.size()<static_cast<std::size_t>(min_points_)) {
      result.rejection="INSUFFICIENT_POINTS";
      return result;
    }
    result.target_reused=indices==target_indices_;
    if(!result.target_reused) {
      target_indices_.clear();
      Cloud::Ptr target(new Cloud);
      target->reserve(indices.size());
      for(const auto i:indices) target->push_back((*map_)[i]);
      registration_.setInputTarget(target);
      score_tree_.setInputCloud(target);
      target_indices_=std::move(indices);
    }
    registration_.setInputSource(source);
    Cloud aligned;
    registration_.align(aligned,guess);
    result.pose=registration_.getFinalTransformation();
    if(!registration_.hasConverged() || !result.pose.allFinite()) {
      result.rejection="NOT_CONVERGED";
      return result;
    }
    std::size_t inliers=0;
    double sum=0.0;
    std::vector<int> idx(1);
    std::vector<float> dist(1);
    for(const auto & p:aligned) {
      if(pcl::isFinite(p) && score_tree_.nearestKSearch(p,1,idx,dist)>0 && dist[0]<=inlier_distance_*inlier_distance_) {
        ++inliers;
        sum+=dist[0];
      }
    }
    result.overlap=double(inliers)/source->size();
    result.error=inliers?std::sqrt(sum/inliers):INFINITY;
    const Matrix delta=guess.inverse()*result.pose;
    result.translation=delta.block<3,1>(0,3).norm();
    result.rotation=Eigen::AngleAxisf(delta.block<3,3>(0,0)).angle();
    pcl::toROSMsg(aligned,result.aligned);
    result.aligned.header=cloud.header;
    result.aligned.header.frame_id=map_frame_;
    result.match_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    return result;
  }

  void match() {
    if(!cloud_ || !odom_) {valid_=false;report("WAITING_FOR_SYNCHRONIZED_INPUT");return;}
    const double age=(now()-rclcpp::Time(cloud_->header.stamp)).seconds();
    if(age<0 || age>max_age_) {valid_=false;report("STALE_INPUT");return;}
    if(!pending_ && !initialized_) {report("WAITING_FOR_INITIALPOSE");return;}
    if(match_future_.valid() || last_processed_==last_input_) return;
    try {
      match_odom_body_=poseMatrix(odom_->pose.pose);
      const Matrix guess=pending_?seed_:map_odom_*match_odom_body_;
      match_epoch_=epoch_;
      match_stamp_=last_input_;
      match_input_age_=age;
      match_first_=pending_ || !initialized_;
      const auto cloud=cloud_;
      match_future_=std::async(std::launch::async,[this,cloud,guess]{return registerCloud(*cloud,guess);});
      last_processed_=last_input_;
    } catch(const std::exception & e) {
      valid_=false;
      report(std::string("GICP_ERROR: ")+e.what());
    }
  }

  void finishMatch() {
    if(!match_future_.valid() ||
       match_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) return;
    MatchResult result;
    try {
      result=match_future_.get();
    } catch(const std::exception & e) {
      if(match_epoch_==epoch_) {valid_=false;report(std::string("GICP_ERROR: ")+e.what());}
      return;
    }
    // A reset or frame error invalidates work captured before it.
    const auto ros_now=now().nanoseconds();
    if(match_epoch_!=epoch_ || (last_clock_ && ros_now<last_clock_)) return;
    if(!result.rejection.empty()) {valid_=false;report(result.rejection);return;}
    const double input_age=(ros_now-match_stamp_)*1e-9;
    const bool accepted=result.overlap>=ratio_ && result.error<=rmse_ &&
      result.translation<=(match_first_?initial_t_:correction_t_) &&
      result.rotation<=(match_first_?initial_r_:correction_r_) && input_age>=0 && input_age<=max_age_;
    valid_=accepted;
    std::ostringstream info;
    info<<(accepted?"ACCEPTED":"REJECTED")<<" rmse_m="<<result.error<<" inlier_ratio="<<result.overlap
      <<" correction_m="<<result.translation<<" correction_rad="<<result.rotation
      <<" match_ms="<<result.match_ms<<" input_age_s="<<input_age<<" input_age_start_s="<<match_input_age_
      <<" target_reused="<<(result.target_reused?"true":"false");
    if(!accepted && input_age>max_age_) info<<" reason=INPUT_EXPIRED_DURING_MATCH";
    if(!accepted) {report(info.str());return;}
    map_odom_=result.pose*match_odom_body_.inverse();
    pending_=false;initialized_=true;
    accepted_stamp_=match_stamp_;
    accepted_pose_=toPose(result.pose);
    report(info.str());
    aligned_pub_->publish(result.aligned);
    geometry_msgs::msg::PoseStamped pose;
    pose.header=result.aligned.header;
    pose.pose=accepted_pose_;
    pose_pub_->publish(pose);
  }
  void broadcast() {
    const auto ros_now=now().nanoseconds();
    if(last_clock_ && ros_now<last_clock_){
      ++epoch_; accepted_stamp_=0; last_input_=0; last_processed_=0;
      pending_=false;initialized_=false;valid_=false;cloud_.reset();odom_.reset();
      report("CLOCK_RESET_NEEDS_INITIALPOSE");
    }
    last_clock_=ros_now;
    if(!valid_) return;
    double age=(now().nanoseconds()-accepted_stamp_)*1e-9;
    if(age<0 || age>ttl_){valid_=false;report("LOCALIZATION_EXPIRED");return;}
    auto p=toPose(map_odom_);geometry_msgs::msg::TransformStamped t;
    t.header.stamp=now();t.header.frame_id=map_frame_;t.child_frame_id=odom_frame_;
    t.transform.translation.x=p.position.x;t.transform.translation.y=p.position.y;t.transform.translation.z=p.position.z;
    t.transform.rotation=p.orientation;broadcaster_->sendTransform(t);
  }
  std::string map_frame_,odom_frame_,body_frame_,source_id_;
  uint64_t epoch_=1,sequence_=0;
  uint64_t cloud_input_count_=0,odom_input_count_=0,paired_input_count_=0;
  int64_t cloud_input_stamp_=0,odom_input_stamp_=0;
  message_filters::Connection cloud_input_connection_,odom_input_connection_;
  int64_t last_clock_=0;
  geometry_msgs::msg::Pose accepted_pose_;
  double voxel_,radius_,period_,max_age_,ttl_,distance_,inlier_distance_,ratio_,rmse_,correction_t_,correction_r_,initial_t_,initial_r_;
  int iterations_,threads_,min_points_;bool pending_=false,initialized_=false,valid_=false;
  int64_t last_input_=0,last_processed_=0,accepted_stamp_=0;
  Matrix seed_=Matrix::Identity(),map_odom_=Matrix::Identity();Cloud::Ptr map_;
  CloudMsg::ConstSharedPtr cloud_;Odom::ConstSharedPtr odom_;
  message_filters::Subscriber<CloudMsg> cloud_sub_;message_filters::Subscriber<Odom> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr map_pub_,aligned_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,input_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<go2_obstacle_interfaces::msg::LocalizationEvidence>::SharedPtr evidence_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr registration_timer_,result_timer_,tf_timer_;
  small_gicp::RegistrationPCL<pcl::PointXYZ,pcl::PointXYZ> registration_;
  pcl::KdTreeFLANN<pcl::PointXYZ> score_tree_;
  std::vector<std::size_t> target_indices_;
  Matrix match_odom_body_=Matrix::Identity();
  uint64_t match_epoch_=0;
  int64_t match_stamp_=0;
  double match_input_age_=0.0;
  bool match_first_=false;
  std::future<MatchResult> match_future_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Localizer>());}
  catch(const std::exception &e){RCLCPP_FATAL(rclcpp::get_logger("gicp_localizer"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;}
