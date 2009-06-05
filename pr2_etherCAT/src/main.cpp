/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <stdio.h>
#include <getopt.h>
#include <execinfo.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <mechanism_control/mechanism_control.h>
#include <ethercat_hardware/ethercat_hardware.h>
#include <urdf/parser.h>

#include <ros/node.h>
#include <std_srvs/Empty.h>
#include <pr2_etherCAT/RealtimeStats.h>

#include <realtime_tools/realtime_publisher.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
using namespace boost::accumulators;

static struct
{
  char *program_;
  char *interface_;
  char *xml_;
  bool allow_override_;
  bool allow_unprogrammed_;
  bool quiet_;
  bool motor_model_;
  bool stats;
} g_options;

void Usage(string msg = "")
{
  fprintf(stderr, "Usage: %s [options]\n", g_options.program_);
  fprintf(stderr, "  Available options\n");
  fprintf(stderr, "    -i, --interface <interface> Connect to EtherCAT devices on this interface\n");
  fprintf(stderr, "    -x, --xml <file|param>      Load the robot description from this file or parameter name\n");
  fprintf(stderr, "    -u, --allow_unprogrammed    Allow control loop to run with unprogrammed devices\n");
  fprintf(stderr, "    -m, --motor_model           Publish motor model values\n");
  fprintf(stderr, "    -s, --stats                 Publish realtime statistics\n");
  fprintf(stderr, "    -q, --quiet                 Don't print warning messages when switching to secondary mode\n");
  fprintf(stderr, "    -h, --help                  Print this message and exit\n");
  if (msg != "")
  {
    fprintf(stderr, "Error: %s\n", msg.c_str());
    exit(-1);
  }
  else
  {
    exit(0);
  }
}

static int g_quit = 0;
static bool g_reset_motors = true;
static bool g_halt_motors = false;
static const int NSEC_PER_SEC = 1e+9;

static struct
{
  accumulator_set<double, stats<tag::max, tag::mean> > ec_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > mc_acc;
  int secondary;
} g_stats;

static void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticMessage> &publisher)
{
  if (publisher.trylock())
  {
    accumulator_set<double, stats<tag::max, tag::mean> > zero;
    vector<diagnostic_msgs::DiagnosticStatus> statuses;
    vector<diagnostic_msgs::DiagnosticValue> values;
    vector<diagnostic_msgs::DiagnosticString> strings;
    diagnostic_msgs::DiagnosticStatus status;
    diagnostic_msgs::DiagnosticValue v;
    diagnostic_msgs::DiagnosticString s;

    static double max_ec = 0, max_mc = 0;
    double avg_ec, avg_mc;

    avg_ec = extract_result<tag::mean>(g_stats.ec_acc);
    avg_mc = extract_result<tag::mean>(g_stats.mc_acc);
    max_ec = std::max(max_ec, extract_result<tag::max>(g_stats.ec_acc)); 
    max_mc = std::max(max_mc, extract_result<tag::max>(g_stats.mc_acc)); 
    g_stats.ec_acc = zero;
    g_stats.mc_acc = zero;

#define ADD_STRING_FMT(lab, fmt, ...) \
  s.label = (lab); \
  { char buf[1024]; \
    snprintf(buf, sizeof(buf), fmt, ##__VA_ARGS__); \
    s.value = buf; \
  } \
  strings.push_back(s)

    ADD_STRING_FMT("Secondary mode switches", "%d", g_stats.secondary);
    ADD_STRING_FMT("Max EtherCAT roundtrip (us)", "%.2f", max_ec*1e+6);
    ADD_STRING_FMT("Avg EtherCAT roundtrip (us)", "%.2f", avg_ec*1e+6);
    ADD_STRING_FMT("Max Mechanism Control roundtrip (us)", "%.2f", max_mc*1e+6);
    ADD_STRING_FMT("Avg Mechanism Control roundtrip (us)", "%.2f", avg_mc*1e+6);

    status.name = "Realtime Control Loop";
    status.level = 0;
    status.message = "OK";
    if (g_stats.secondary > 100) {
      status.level = 1;
      status.message = "Too many secondary mode switches";
    }
    if (g_stats.secondary > 1000) {
      status.level = 2;
      status.message = "Too many secondary mode switches";
    }

    status.set_values_vec(values);
    status.set_strings_vec(strings);
    statuses.push_back(status);
    publisher.msg_.set_status_vec(statuses);
    publisher.unlockAndPublish();
  }
}

static inline double now()
{
  struct timespec n;
  clock_gettime(CLOCK_MONOTONIC, &n);
  return double(n.tv_nsec) / NSEC_PER_SEC + n.tv_sec;
}

void *controlLoop(void *)
{
  realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticMessage> publisher("/diagnostics", 2);
  realtime_tools::RealtimePublisher<pr2_etherCAT::RealtimeStats> *rtpublisher = 0;
  accumulator_set<double, stats<tag::max, tag::mean> > acc;

  if (g_options.stats)
  {
    rtpublisher = new realtime_tools::RealtimePublisher<pr2_etherCAT::RealtimeStats> ("/realtime", 2);
  }

  // Publish one-time before entering real-time to pre-allocate message vectors
  publishDiagnostics(publisher);

  // Initialize the hardware interface
  EthercatHardware ec;
  ec.init(g_options.interface_, g_options.allow_unprogrammed_, g_options.motor_model_);

  // Create mechanism control
  MechanismControl mc(ec.hw_);
  MechanismControlNode mcn(&mc);

  // Load robot description
  TiXmlDocument xml;
  struct stat st;
  if (0 == stat(g_options.xml_, &st))
  {
    xml.LoadFile(g_options.xml_);
  }
  else
  {
    ROS_INFO("Xml file not found, reading from parameter server\n");
    assert(ros::Node::instance());
    std::string result;
    if (ros::Node::instance()->getParam(g_options.xml_, result))
      xml.Parse(result.c_str());
    else
    {
      ROS_FATAL("Could not load the xml from parameter server: %s\n", g_options.xml_);
      exit(1);
    }
  }
  TiXmlElement *root_element = xml.RootElement();
  TiXmlElement *root = xml.FirstChildElement("robot");
  if (!root || !root_element)
  {
      ROS_FATAL("Could not parse the xml from %s\n", g_options.xml_);
      exit(1);
  }
  urdf::normalizeXml(root_element);

  // Register actuators with mechanism control
  ec.initXml(root, g_options.allow_override_);

  // Initialize mechanism control from robot description
  mcn.initXml(root);

  // Set to realtime scheduler for this thread
  struct sched_param thread_param;
  int policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);

  struct timespec tick;
  clock_gettime(CLOCK_MONOTONIC, &tick);
  int period = 1e+6; // 1 ms in nanoseconds

  double last_published = now();
  while (!g_quit)
  {
    double start = now();
    if (g_reset_motors)
    {
      accumulator_set<double, stats<tag::max, tag::mean> > zero;
      acc = zero;
      ec.update(true, g_halt_motors);
      g_reset_motors = false;
    }
    else
    {
      ec.update(false, g_halt_motors);
    }
    g_halt_motors = false;
    double after_ec = now();
    mcn.update();
    double end = now();

    g_stats.ec_acc(after_ec - start);
    g_stats.mc_acc(end - after_ec);

    if ((end - last_published) > 1.0)
    {
      publishDiagnostics(publisher);
      last_published = end;
    }

    tick.tv_nsec += period;
    while (tick.tv_nsec >= NSEC_PER_SEC)
    {
      tick.tv_nsec -= NSEC_PER_SEC;
      tick.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, NULL);

    if (rtpublisher)
    {
      struct timespec after;
      clock_gettime(CLOCK_MONOTONIC, &after);
      double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec-tick.tv_nsec)/1e9)*1e6;
      acc(jitter);
      if (rtpublisher->trylock())
      {
        rtpublisher->msg_.jitter  = jitter;
        rtpublisher->msg_.avg_jitter  = extract_result<tag::mean>(acc);
        rtpublisher->msg_.max_jitter  = extract_result<tag::max>(acc);
        rtpublisher->unlockAndPublish();
      }
    }
  }

  /* Shutdown all of the motors on exit */
  for (unsigned int i = 0; i < ec.hw_->actuators_.size(); ++i)
  {
    ec.hw_->actuators_[i]->command_.enable_ = false;
    ec.hw_->actuators_[i]->command_.effort_ = 0;
  }
  ec.update(false, true);

  publisher.stop();
  if (rtpublisher) delete rtpublisher;

  return 0;
}

void quitRequested(int sig)
{
  g_quit = 1;
}

bool shutdownService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  quitRequested(0);
  return true;
}

bool resetMotorsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_reset_motors = true;
  return true;
}

bool haltMotorsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  g_halt_motors = true;
  return true;
}

void warnOnSecondary(int sig)
{
  void *bt[32];
  int nentries;

  // Increment diagnostic count of secondary mode switches
  ++g_stats.secondary;

  // Dump a backtrace of the frame which caused the switch to
  // secondary mode
  if (!g_options.quiet_)
  {
    nentries = backtrace(bt, sizeof(bt) / sizeof(bt[0]));
    backtrace_symbols_fd(bt, nentries, fileno(stdout));
    printf("\n");
    fflush(stdout);
  }
}

static int
lock_fd(int fd)
{
  struct flock lock;
  int rv;

  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  lock.l_start = 0;
  lock.l_len = 0;

  rv = fcntl(fd, F_SETLK, &lock);
  return rv;
}

#define PIDFILE "/var/run/pr2_etherCAT.pid"
static int setupPidFile(void)
{
  int rv = -1;
  pid_t pid;
  int fd;
  FILE *fp = NULL;

  fd = open(PIDFILE, O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
  if (fd == -1)
  {
    if (errno != EEXIST)
    {
      ROS_FATAL("Unable to create pid file '%s': %s", PIDFILE, strerror(errno));
      goto end;
    }

    if ((fd = open(PIDFILE, O_RDWR)) < 0)
    {
      ROS_FATAL("Unable to open pid file '%s': %s", PIDFILE, strerror(errno));
      goto end;
    }

    if ((fp = fdopen(fd, "rw")) == NULL)
    {
      ROS_FATAL("Can't read from '%s': %s", PIDFILE, strerror(errno));
      goto end;
    }
    pid = -1;
    if ((fscanf(fp, "%d", &pid) != 1) || (pid == getpid()) || (lock_fd(fileno(fp)) == 0))
    {
      int rc;

      if ((rc = unlink(PIDFILE)) == -1)
      {
        ROS_FATAL("Can't remove stale pid file '%s': %s", PIDFILE, strerror(errno));
        goto end;
      }
    } else {
      ROS_FATAL("Another instance of pr2_etherCAT is already running with pid: %d\n", pid);
      goto end;
    }
  }

  unlink(PIDFILE);
  fd = open(PIDFILE, O_RDWR | O_CREAT | O_EXCL, S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);

  if (fd == -1)
  {
    ROS_FATAL("Unable to open pid file '%s': %s", PIDFILE, strerror(errno));
    goto end;
  }

  if (lock_fd(fd) == -1)
  {
    ROS_FATAL("Unable to lock pid file '%s': %s", PIDFILE, strerror(errno));
    goto end;
  }

  if ((fp = fdopen(fd, "w")) == NULL)
  {
    ROS_FATAL("fdopen failed: %s", strerror(errno));
    goto end;
  }

  fprintf(fp, "%d\n", getpid());

  /* We do NOT close fd, since we want to keep the lock. */
  fflush(fp);
  fcntl(fd, F_SETFD, (long) 1);
  rv = 0;
end:
  return rv;
}

static void cleanupPidFile(void)
{
  unlink(PIDFILE);
}

#define CLOCK_PRIO 0
#define CONTROL_PRIO 0

static pthread_t controlThread;
static pthread_attr_t controlThreadAttr;
int main(int argc, char *argv[])
{
  // Must run as root
  if (geteuid() != 0)
  {
    fprintf(stderr, "You must run as root!\n");
    exit(-1);
  }

  // Keep the kernel from swapping us out
  mlockall(MCL_CURRENT | MCL_FUTURE);

  // Setup single instance
  if (setupPidFile() < 0) return -1;

  // Initialize ROS and parse command-line arguments
  ros::init(argc, argv);

  // Parse options
  g_options.program_ = argv[0];
  while (1)
  {
    static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"allow_override", no_argument, 0, 'a'},
      {"allow_unprogrammed", no_argument, 0, 'u'},
      {"quiet", no_argument, 0, 'q'},
      {"motor_model", no_argument, 0, 'm'},
      {"stats", no_argument, 0, 's'},
      {"interface", required_argument, 0, 'i'},
      {"xml", required_argument, 0, 'x'},
    };
    int option_index = 0;
    int c = getopt_long(argc, argv, "ahi:mqsux:", long_options, &option_index);
    if (c == -1) break;
    switch (c)
    {
      case 'h':
        Usage();
        break;
      case 'a':
        g_options.allow_override_ = 1;
        break;
      case 'u':
        g_options.allow_unprogrammed_ = 1;
        break;
      case 'm':
        g_options.motor_model_ = 1;
        break;
      case 'q':
        g_options.quiet_ = 1;
        break;
      case 's':
        g_options.stats = 1;
        break;
      case 'i':
        g_options.interface_ = optarg;
        break;
      case 'x':
        g_options.xml_ = optarg;
        break;
    }
  }
  if (optind < argc)
  {
    Usage("Extra arguments");
  }

  if (!g_options.interface_)
    Usage("You must specify a network interface");
  if (!g_options.xml_)
    Usage("You must specify a robot description XML file");

  ros::Node *node = new ros::Node("pr2_etherCAT",
                                  ros::Node::DONT_HANDLE_SIGINT);

  // Catch attempts to quit
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  // Catch if we fall back to secondary mode
  signal(SIGXCPU, warnOnSecondary);

  node->advertiseService("shutdown", shutdownService);
  node->advertiseService("reset_motors", resetMotorsService);
  node->advertiseService("halt_motors", haltMotorsService);

  //Start thread
  int rv;
  if ((rv = pthread_create(&controlThread, &controlThreadAttr, controlLoop, 0)) != 0)
  {
    ROS_FATAL("Unable to create control thread: rv = %d\n", rv);
    ROS_BREAK();
  }

  pthread_join(controlThread, 0);

  // Cleanup pid file
  cleanupPidFile();

  delete node;

  return 0;
}
