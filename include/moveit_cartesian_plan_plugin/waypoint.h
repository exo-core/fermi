//
// Created by arne on 22.01.18.
//

#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <string.h>
#include <tf/tf.h>

namespace moveit_cartesian_plan_plugin {
	class Waypoint {
		public:
			Waypoint(const std::string& name) ;
			Waypoint(const std::string& name, const tf::Transform& pose) ;
			virtual ~Waypoint() ;

			Waypoint& operator=(const Waypoint& other) ;
			bool operator ==(const Waypoint &other) const;

			std::string name_;
			tf::Transform pose_;
	};
}

#endif //WAYPOINT_H
