#include "john_controls/thrust_allocator.hpp"
#include <unistd.h>
using std::placeholders::_1;

namespace john_controls
{

  ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions & options)
  : Node("thrust_allocator", options),
    x_lens_ (MAX_THRUSTERS, 0),
    y_lens_ (MAX_THRUSTERS, 0),
    z_lens_ (MAX_THRUSTERS, 0),
    x_contribs_ (MAX_THRUSTERS, 0),
    y_contribs_ (MAX_THRUSTERS, 0),
    z_contribs_ (MAX_THRUSTERS, 0)
  {
      this->declare_parameter<int>("num_thrusters", num_thrusters_);

      this->get_parameter("num_thrusters", num_thrusters_);
      if (num_thrusters_ > MAX_THRUSTERS || num_thrusters_ <= 0){
        RCLCPP_ERROR(this->get_logger(), "Attempted to configure too many or not enough thrusters, results will not be desirable");
      } 

    // REMOVE EXPLANATION COMMENTS LATER bits per thruster determines resolution of thruster contorl signals
      this->declare_parameter<int>("bits_per_thruster", bits_per_thruster_);
      this->get_parameter("bits_per_thruster", bits_per_thruster_);

    // max forward and reverse thrust levels, confvert to newtons
      this->declare_parameter<double>("max_fwd", max_fwd_);
      this->get_parameter("max_fwd", max_fwd_);
      max_fwd_ *= 9.807; // kgf to N
      this->declare_parameter<double>("max_rev", max_rev_);
      max_rev_ *= 9.807; // kgf to N
      this->get_parameter("max_rev", max_rev_);

        // encode levels mean thatminim reverse thrust to max forward thrust into discrete levels
      encode_levels_ = pow(2,bits_per_thruster_);

      std::string names[MAX_THRUSTERS] = {"t1", "t2", "t3", "t4", "t5", "t6"};
    
      for (int i = 0; i < MAX_THRUSTERS; i++)
      {
        // store contribution to each thrsuter among respective axis
        this->declare_parameter(names[i]+".contrib.x", x_contribs_[i]);
        this->declare_parameter(names[i]+".contrib.y", y_contribs_[i]);
        this->declare_parameter(names[i]+".contrib.z", z_contribs_[i]);
        this->declare_parameter(names[i]+".lx", x_lens_[i]);
        this->declare_parameter(names[i]+".ly", y_lens_[i]);
        this->declare_parameter(names[i]+".lz", z_lens_[i]);

        this->get_parameter(names[i]+".contrib.x", x_contribs_[i]);
        this->get_parameter(names[i]+".contrib.y", y_contribs_[i]);
        this->get_parameter(names[i]+".contrib.z", z_contribs_[i]);
        this->get_parameter(names[i]+".lx", x_lens_[i]);
        this->get_parameter(names[i]+".ly", y_lens_[i]);
        this->get_parameter(names[i]+".lz", z_lens_[i]);
      } 

    //compute allocation matrix, each row represents force/torque axis and each column represents
    // thruster's contirbution
      std::vector<std::vector<double>> alloc_vec =  createAllocMat();

      cv::Mat alloc_mat (alloc_vec.size(), alloc_vec[0].size(), CV_64FC1);
      for (int i = 0; i < alloc_mat.rows; i++)
        for (int j = 0; j < alloc_mat.cols; j++)
          alloc_mat.at<double>(i,j) = alloc_vec[i][j];
      
      RCLCPP_INFO(this->get_logger(), "Allocation Matrix:");
      std::cout << alloc_mat << std::endl;

        // opencv, computes SVD decomp
        // useful for over or under actuated
        /**
        overactuated: more thrsuters then degrees of freedom
        underactuated: less thrusters than degrees of freedom
        f = AT, f is desired force, A is allocation matrix, T is thruster force vector (output)
        So we take pseudo inverse to compute T. Given A is non-square (under or over)
        
         */
      cv::invert(alloc_mat, pinv_alloc_, cv::DECOMP_SVD);

    // publish the forces, output forces is the forces for each
      forces_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "output_forces", 10);

    // encoded signals, maybe combine into one
      signals_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
        "signals", 10);

      forces_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "input_forces", 10, std::bind(&ThrustAllocator::computeThrustFromWrench, this, _1));

      RCLCPP_INFO(this->get_logger(), "Thrust Allocator succesfully started!");
  }

    void ThrustAllocator::computeThrustFromWrench(const geometry_msgs::msg::Wrench::SharedPtr msg) const
    { 
    // waits for input forces, coming in from navigation
    // takes in input forcesand torque, 6 by 1 matrix
    // mutliplied by pseudo inversde to compute thrusts
    if (std::isnan(msg->force.x) || std::isnan(msg->force.y) || std::isnan(msg->force.z) ||
        std::isnan(msg->torque.x) || std::isnan(msg->torque.y) || std::isnan(msg->torque.z)) {
        std::cout << "INVALID INPUT FORCES " << endl;
        return;
    }
    std::vector<double> tau_vec = {msg->force.x, msg->force.y, msg->force.z,
                               msg->torque.x, msg->torque.y, msg->torque.z};
    
    cv::Mat tau_mat(tau_vec); 

    cv::Mat thrust_mat =  pinv_alloc_*tau_mat;

    std::vector<double> thrust;
    uint32_t signal = 0;
    for (int i = 0; i < num_thrusters_ ; i++)
    {
        double thruster_thrust = thrust_mat.at<double>(i,0);
        thrust.push_back(thruster_thrust);

        // force to level handles conversion (digital levels, discretizaed signals)
        uint32_t t_level = forceToLevel(thruster_thrust);
        uint32_t t_bits = t_level << (bits_per_thruster_ * i);
        // std::cout << "thruster " << i << " " << thruster_thrust << " " << t_bits  << " " << (t_bits >> bits_per_thruster_ * i) << std::endl;
        signal |= t_bits;
    }
    //combined to 32 bit integer signal, intended for the teensy

    // TODO: PLEASE CHANGE ME LATER
    // so that the teensy can distinguish between rubbish and real data
    signal |= 0b10101010000000000000000000000000;

    std::cout << "Thrust values: ";
    for (const auto& t : thrust) {
        std::cout << t << " ";
    }
    std::cout << "\nSignal: " << signal << std::endl;

    auto forces_msg = std_msgs::msg::Float64MultiArray();
    forces_msg.data = thrust;
    forces_pub_->publish(forces_msg);
    auto signal_msg = std_msgs::msg::UInt32();
    signal_msg.data = signal;
    signals_pub_->publish(signal_msg);
    }

    std::vector<double> ThrustAllocator::cross(std::vector<double> r,std::vector<double> F){
      std::vector<double> tau;
      tau.push_back(r[1]*F[2] - r[2]*F[1]);
      tau.push_back(r[2]*F[0] - r[0]*F[2]);
      tau.push_back(r[0]*F[1] - r[1]*F[0]);
      return tau;
    }

    std::vector<std::vector<double>> ThrustAllocator::createAllocMat(){
      std::vector<std::vector<double>> alloc_mat;
      for (int i = 0; i < num_thrusters_; i++){
        std::vector<double> F = {x_contribs_[i], y_contribs_[i], z_contribs_[i]};
        std::vector<double> r = {x_lens_[i], y_lens_[i], z_lens_[i]};
        std::vector<double> tau = cross(r, F);
        F.insert(F.end(), tau.begin(), tau.end());
        alloc_mat.push_back(F);
      }
      std::vector<std::vector<double>> alloc_mat_trans(6, std::vector<double>(num_thrusters_));
      for(int i = 0; i < num_thrusters_; ++i){
        for(int j = 0; j < 6; ++j){
            alloc_mat_trans[j][i]=alloc_mat[i][j];
        }
      }
      return alloc_mat_trans;
    }


    uint32_t ThrustAllocator::forceToLevel(double force) const {
        const double FORCE_THRESHOLD = 0.1; // configurable threshold
        const uint32_t LEVELS_MIDPOINT = encode_levels_ / 2; // midpoint for positive/negative levels
        const uint32_t MAX_LEVEL = encode_levels_ - 1; // max possible level value

        // threshold small forces to zero
        if (std::abs(force) < FORCE_THRESHOLD) {
            force = 0;
        }

        uint32_t t_level;
        if (force >= 0) {
            // positive forces: levels range from LEVELS_MIDPOINT to MAX_LEVEL
            t_level = static_cast<uint32_t>(std::round(std::min(force, max_fwd_) / max_fwd_ * (LEVELS_MIDPOINT - 1)));
            t_level += LEVELS_MIDPOINT;
        } else {
            // negative forces: levels range from 0 to LEVELS_MIDPOINT - 1
            t_level = static_cast<uint32_t>(LEVELS_MIDPOINT - std::round(std::max(force, -max_rev_) / -max_rev_ * LEVELS_MIDPOINT));
        }

        // apply bitmask to ensure result fits within thruster encoding
        t_level &= ((1U << bits_per_thruster_) - 1);

        return t_level;
    }
    
} // namespace john_controls


int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions();
    rclcpp::spin(std::make_shared<john_controls::ThrustAllocator>(options));
    rclcpp::shutdown();
  } catch (rclcpp::exceptions::RCLError const& ex){
    std::cerr << "RCLError caught: " << ex.what() << std::endl;
  } // during testing sometimes throws error
  return 0;
}