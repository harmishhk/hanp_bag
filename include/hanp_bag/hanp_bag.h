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
 * CONTRACT,1 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *                                  Harmish Khambhaita on Thu Nov 10 2016
 */

#ifndef HANP_BAG_H_
#define HANP_BAG_H_

#include <ros/ros.h>

#include <hanp_bag/SetString.h>
#include <std_srvs/Trigger.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace hanp_bag {
class HANPBag {
public:
  HANPBag();
  ~HANPBag();

  void init();

private:
  // ros services
  ros::ServiceServer get_bags_srv_, set_bag_srv_;

  std::string get_bags_srv_n_, set_bag_srv_n_;

  std::string bags_path_, bag_file_;

  bool getBags(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res);
  bool setBag(hanp_bag::SetString::Request &req,
              hanp_bag::SetString::Response &res);
};
}

#endif // HANP_BAG_H_
