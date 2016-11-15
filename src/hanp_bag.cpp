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
#define SET_BAGFILE_SRV "/set_bag_file"

#include <signal.h>

#include <hanp_bag/hanp_bag.h>

namespace hanp_bag {
// empty constructor and destructor
HANPBag::HANPBag() {}
HANPBag::~HANPBag() {}

void HANPBag::init() {
  // get private node handle
  ros::NodeHandle nh("~/");

  // get parameters
  nh.param("set_bagfile_srv", set_bagfile_srv_n_, std::string(SET_BAGFILE_SRV));

  // initialize services
  set_bagfile_srv_ =
      nh.advertiseService(set_bagfile_srv_n_, &HANPBag::setBagFile, this);
}

bool HANPBag::setBagFile(hanp_bag::SetString::Request &req,
                         hanp_bag::SetString::Response &res) {
  rosbag::Bag bag;
  try {
    bag.open(req.data, rosbag::bagmode::Read);
    bag.close();
    res.success = true;
  } catch (rosbag::BagException ex) {
    res.message = std::string("Cannot open bag file, ") + ex.what();
    res.success = true;
    ROS_ERROR_NAMED(NODE_NAME, "%s", res.message.c_str());
  }

  return res.success;
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
