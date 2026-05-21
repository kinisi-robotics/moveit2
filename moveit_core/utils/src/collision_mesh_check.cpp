/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Kinisi Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kinisi Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/utils/collision_mesh_check.hpp>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace moveit
{

bool strictCollisionMeshes()
{
  static const bool strict = []() {
    const char* env = std::getenv("MOVEIT_STRICT_COLLISION_MESHES");
    if (!env)
      return true;
    std::string val(env);
    std::transform(val.begin(), val.end(), val.begin(), ::tolower);
    return val != "0" && val != "false" && val != "no" && val != "off";
  }();
  return strict;
}

void handleCollisionMeshFailure(const std::string& filename)
{
  const bool strict = strictCollisionMeshes();
  const char* severity = strict ? "FATAL" : "WARNING";

  std::string msg = std::string("[") + severity + "] Failed to load collision mesh: " + filename +
                    "\n        The robot cannot operate without collision geometry."
                    "\n        Set MOVEIT_STRICT_COLLISION_MESHES=0 to downgrade to a warning.\n";
  std::cerr << "\n" << msg << std::endl;

  const std::string path = "/tmp/moveit_fatal_collision_mesh.log";
  std::ofstream f(path, std::ios::app);
  if (f.is_open())
  {
    f << msg;
    std::cerr << "        (details written to " << path << ")" << std::endl;
  }

  if (strict)
  {
    throw std::runtime_error("Failed to load collision mesh: " + filename +
                             ". The robot cannot operate without collision geometry."
                             " Set MOVEIT_STRICT_COLLISION_MESHES=0 to downgrade to a warning.");
  }
}

}  // namespace moveit
