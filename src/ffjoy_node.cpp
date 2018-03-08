#include <linux/input.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#define BITS_PER_LONG (sizeof(long) * 8)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)    ((array[LONG(bit)] >> OFF(bit)) & 1)

int fd; // file descriptor

template<typename T> inline T clamp(T Value, const T Min, const T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

//! strong and weak in [0, 1]
bool rumble(double strong, double weak, double dur_seconds, bool sleep = true) {
  // Uploading an Effect
  struct ff_effect effect;
  effect.type = FF_RUMBLE;
  effect.id = -1;
  effect.u.rumble.strong_magnitude = 65536 * clamp(strong, 0., 1.);
  effect.u.rumble.weak_magnitude   = 65536 * clamp(weak, 0., 1.);
  effect.replay.length = 1000 * clamp(dur_seconds, 0., 5.);
  effect.replay.delay  = 0;
  if (ioctl(fd, EVIOCSFF, &effect) == -1) {
    ROS_ERROR("upload effect");
    return false;
  }

  // Playing/Stopping an Effect
  struct input_event play;
  play.type = EV_FF;
  play.code =  effect.id; /* the id we got when uploading the effect */
  play.value = 1; /* play: 1, stop: 0 */
  if (write(fd, (const void*) &play, sizeof(play)) == -1) {
    ROS_ERROR("sending event");
    return false;
  }
  if (sleep)
    usleep(dur_seconds * 1E6);
  return true;
}

void cb(const std_msgs::Float64MultiArray & msg) {
  if (msg.data.size() != 3) {
    ROS_WARN("Message should contain three fields: strong(0~1), weak(0~1), duration");
    return;
  }
  rumble(msg.data[0], msg.data[1], msg.data[2]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ffjoy_node");
  ros::NodeHandle nh_private("~");
  std::string device = "/dev/input/event1";
  nh_private.param("device", device, device);

  // https://freegamedev.net/wiki/Force_Feedback
  fd = open(device.c_str(), O_RDWR);
  if (!fd) {
    ROS_FATAL("Could not open device '%s'", device.c_str());
    return -1;
  }

  /* Number of effects the device can play at the same time */
  int n_effects;
  if (ioctl(fd, EVIOCGEFFECTS, &n_effects) == -1) {
    ROS_ERROR("Ioctl query error on device '%s'", device.c_str());
  }
  else
    ROS_INFO("Device '%s' supports %i effects at the same time", device.c_str(), n_effects);
  // Querying device capabilites
  unsigned long features[4];
  if (ioctl(fd, EVIOCGBIT(EV_FF, sizeof(features)), features) == -1) {
    ROS_ERROR("Ioctl query on device '%s'", device.c_str());
  }
  if (test_bit(FF_CONSTANT, features)) ROS_INFO("Constant supported");
  if (test_bit(FF_PERIODIC, features)) ROS_INFO("Periodic supported");
  if (test_bit(FF_SPRING, features))   ROS_INFO("Spring supported");
  if (test_bit(FF_FRICTION, features)) ROS_INFO("Friction supported");
  if (test_bit(FF_RUMBLE, features))   ROS_INFO("Rumble supported");

  ros::Subscriber sub = nh_private.subscribe("cmd", 1, cb);
  ros::spin();
  close(fd);
  return 0;
}
