/*/
 * Copyright (c) 2016 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Thu Nov 10 2016
 */

// defining constants
#define NODE_NAME "hanp_bag"

#define BAGS_PATH "/tmp/bags"
#define GET_BAGS_SRV "get_bag_files"
#define SET_BAG_SRV "set_bag_file"

#include <hanp_bag/hanp_bag.h>

#include <signal.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace hanp_bag {
// empty constructor and destructor
HANPBag::HANPBag() {}
HANPBag::~HANPBag() {}

void HANPBag::init() {
  // get private node handle
  ros::NodeHandle nh("~/");

  // get parameters
  nh.param("bags_path", bags_path_, std::string(BAGS_PATH));
  nh.param("get_bags_srv", get_bags_srv_n_, std::string(GET_BAGS_SRV));
  nh.param("set_bag_srv", set_bag_srv_n_, std::string(SET_BAG_SRV));

  // initialize services
  get_bags_srv_ = nh.advertiseService(get_bags_srv_n_, &HANPBag::getBags, this);
  set_bag_srv_ = nh.advertiseService(set_bag_srv_n_, &HANPBag::setBag, this);
}

bool HANPBag::getBags(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res) {
  fs::path p(bags_path_);
  try {
    if (fs::exists(p) && fs::is_directory(p)) {
      for (auto &bag_f : boost::make_iterator_range(fs::directory_iterator(p),
                                                    fs::directory_iterator())) {
        if (fs::is_regular_file(bag_f) && bag_f.path().extension() == ".bag") {
          res.message += bag_f.path().filename().string() + std::string(";");
        }
      }
      if (res.message != "") {
        res.success = true;
        res.message.pop_back();
      }
    } else {
      res.success = false;
      res.message =
          std::string("Bags directory %s does not exist ") + bags_path_;
      ROS_ERROR_NAMED(NODE_NAME, "%s", res.message.c_str());
    }
  } catch (const fs::filesystem_error &ex) {
    res.success = false;
    res.message = std::string("Filesystem error: ") + ex.what();
    ROS_ERROR_NAMED(NODE_NAME, "%s", res.message.c_str());
  }
  return true;
}

bool HANPBag::setBag(hanp_bag::SetString::Request &req,
                     hanp_bag::SetString::Response &res) {
  rosbag::Bag bag;
  try {
    bag.open(req.data, rosbag::bagmode::Read);
    bag.close();
    res.success = true;
  } catch (const rosbag::BagException &ex) {
    res.message = std::string("Cannot open bag file, ") + ex.what();
    res.success = true;
    ROS_ERROR_NAMED(NODE_NAME, "%s", res.message.c_str());
  }
  return true;
}
} // namespace hanp_bag

// handler for something to do before killing the node
void sigintHandler(int sig) {
  ROS_DEBUG_NAMED(NODE_NAME, "Node %s will now shutdown", NODE_NAME);

  // the default sigint handler, it calls shutdown() on node
  ros::shutdown();
}

// the main method starts a rosnode and initializes the HANPBag class
int main(int argc, char **argv) {
  // starting the hanp_bag node
  ros::init(argc, argv, NODE_NAME);
  ROS_DEBUG_NAMED(NODE_NAME, "Started %s node", NODE_NAME);

  // initiazling HANPBag class
  hanp_bag::HANPBag hANPBag;
  hANPBag.init();

  // look for sigint and start spinning the node
  signal(SIGINT, sigintHandler);
  ros::spin();

  return 0;
}
