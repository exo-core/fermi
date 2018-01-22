//
// Created by arne on 22.01.18.
//

#include "../include/moveit_cartesian_plan_plugin/waypoint.h"

namespace moveit_cartesian_plan_plugin {
	Waypoint::Waypoint(const std::string& name)
	: name_(name) {
	};

	Waypoint::Waypoint(const std::string& name, const tf::Transform& pose)
	: name_(name),
	  pose_(pose) {

	}

	bool Waypoint::operator ==(const Waypoint &other) const {
		return name_ == other.name_ && pose_ == other.pose_ ;
	}

	Waypoint::~Waypoint() {
	}
}