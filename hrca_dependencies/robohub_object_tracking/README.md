# Under Development

Software packages made for tracking objects in the robohub with either Aruco markers, Vicon, or custom messages.

.

# Classes needed for import
(see the demo package for an example of use):

* TrackingSystem: keeps track of objects, responds to vicon/aruco/custom detection messages
* TrackedObject: represents an object. Has a base pose for some theoretical point

# Other Concepts:

## Query Point

Many (or none) of these, identified by a string name, can belong to an object. They are given a pose relative to the object's base pose. You can query this pose, and it will be kept updated as the object is detected in new poses.