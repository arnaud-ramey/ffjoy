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
struct ff_effect effect;
bool was_uploaded_once = false; // true after the first upload of "effect" to the joy

template<typename T> inline T clamp(T Value, const T Min, const T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

bool open_device(const std::string & device,
                 bool print_errors = true) {
  ROS_DEBUG("Probing device '%s'...", device.c_str());
  // check device exists
  struct stat buffer;
  if (stat(device.c_str(), &buffer) != 0) {
    ROS_ERROR("Device '%s' does not exist", device.c_str());
    return false;
  }
  fd = open(device.c_str(), O_RDWR);
  if (!fd) {
    if (print_errors) ROS_ERROR("Could not open device '%s'", device.c_str());
    return false;
  }
  /* Number of effects the device can play at the same time */
  int n_effects;
  if (ioctl(fd, EVIOCGEFFECTS, &n_effects) == -1) {
    if (print_errors) ROS_ERROR("Could not get number of effects on device '%s'", device.c_str());
    return false;
  }
  if (n_effects == 0) {
    if (print_errors) ROS_ERROR("Device '%s' does not support effects", device.c_str());
    return false;
  }
  ROS_INFO("Device '%s' supports %i effects at the same time", device.c_str(), n_effects);

  // Querying device capabilites
  unsigned long features[4];
  if (ioctl(fd, EVIOCGBIT(EV_FF, sizeof(features)), features) == -1) {
    if (print_errors) ROS_ERROR("Ioctl query on device '%s'", device.c_str());
    return false;
  }
  if (test_bit(FF_CONSTANT, features)) ROS_INFO(" - Constant supported");
  if (test_bit(FF_DAMPER, features))   ROS_INFO(" - Damper supported");
  if (test_bit(FF_FRICTION, features)) ROS_INFO(" - Friction supported");
  if (test_bit(FF_INERTIA, features))  ROS_INFO(" - Inertia supported");
  if (test_bit(FF_PERIODIC, features)) ROS_INFO(" - Periodic supported");
  if (test_bit(FF_RAMP, features))     ROS_INFO(" - Ramp supported");
  if (test_bit(FF_RUMBLE, features))   ROS_INFO(" - Rumble supported");
  if (test_bit(FF_SPRING, features))   ROS_INFO(" - Spring supported");
  return true;
}

bool find_auto_device() {
  ROS_INFO("Searching a Force Feedback device in /dev/input/event*...");
  for (int device_id = 0; device_id < 50; ++device_id) {
    std::ostringstream device; device << "/dev/input/event" << device_id;
    if (open_device(device.str(), false))
      return true;
  }
  ROS_FATAL("Found no Force Feedback device in /dev/input/event* !");
  return false;
}

//! strong and weak in [0, 1]
bool rumble(double strong, double weak, double dur_seconds, bool sleep = true) {
  // Upload the Effect
  effect.type = FF_RUMBLE;
  if (!was_uploaded_once) // otherwise we re-use the same ID
    effect.id = -1;
  effect.u.rumble.strong_magnitude = 65535 * clamp(strong, 0., 1.);
  effect.u.rumble.weak_magnitude   = 65535 * clamp(weak, 0., 1.);
  effect.replay.length = 1000 * clamp(dur_seconds, 0., 5.);
  effect.replay.delay  = 0;
  if (ioctl(fd, EVIOCSFF, &effect) == -1) {
    ROS_ERROR("Error at uploading FF effect");
    return false;
  }
  was_uploaded_once = true;
  //ROS_INFO("Effect ID after uploading:%i", effect.id);

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
  std::string device = "auto";
  nh_private.param("device", device, device);
  if (device == "auto") { // find a FF-compatible device
    if (!find_auto_device()) {
      ros::shutdown();
      return -1;
    }
  }
  else if (!open_device(device)) { // open the device with the given name
    ros::shutdown();
    return -1;
  }

  ros::Subscriber sub = nh_private.subscribe("cmd", 1, cb);
  ros::spin();
  // clean - try to remove the effect
  if (was_uploaded_once && ioctl(fd, EVIOCRMFF, &effect) == -1) {
    ROS_ERROR("Error at removing FF effect");
    return false;
  }
  close(fd);
  return 0;
}
