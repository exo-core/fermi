//
// Created by arne on 22.01.18.
//

#ifndef PROJECT_WAYPOINT_H
#define PROJECT_WAYPOINT_H

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

#endif //PROJECT_WAYPOINT_H
