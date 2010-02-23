// EML
#include <ethercat/netif.h>
#include <ethercat/ethercat_xenomai_drv.h>
#include <dll/ethercat_frame.h>
#include <dll/ethercat_device_addressed_telegram.h>

// STL
#include <vector>
#include <stdexcept>
#include <iostream>
#include <string>
#include <sstream>

// BOOST
#include <boost/foreach.hpp>

// ROS
#include <ros/ros.h>
#include <ectools/ecstats.h>

// C
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

using std::cout;
using std::cerr;
using std::endl;
using std::ostringstream;
using std::string;
using std::runtime_error;

// from getopt
extern char *optarg;
extern int optind, opterr, optopt;

static const unsigned default_packet_size = 1400;
static const unsigned default_num_threads = 8;

void usage(char const *progname) {
  fprintf(
          stderr,
          "usage: %s [-i <interface>] [-s <size>] [-d <delay>] [-l <preload>] [-v]\n"
          " Measures Ethernet bandwidth and packet loss.  Also, counts number of EtherCAT devices.\n"
          " Publishes results on 'ecstats' topic.\n"
          " -i : network interface to use when communicating with EtherCAT MCBs\n"
          " -s : <size> of packet to send.  Defaults to %d\n"
          " -j : number of threads to run.  Defaults to %d\n"
          " -v : increase verbosity\n"
          " -h : display this help\n",
          progname, default_packet_size, default_num_threads);
}

int get_request_sock() throw (std::exception)
{
  static int request_sock = -1; 
  if (request_sock < 0) {    
    // create generic socket to use for ioctl
    request_sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (request_sock < 0) {
      int error = errno;
      std::ostringstream os;
      os << "Couldn't open generic socket for ioctls " << strerror(error); 
      throw std::runtime_error(os.str());
    }
  }  
  return request_sock;
}
  

// returns true if interface is up
bool is_iface_up(const char* iface_name) throw (std::exception)
{
  int request_sock = get_request_sock();

  // get interface flags
  struct ifreq ifr;                 
  strncpy(ifr.ifr_name, iface_name, IFNAMSIZ);
  if (ioctl(request_sock, SIOCGIFFLAGS, &ifr) < 0) {
    int error = errno;
    close(request_sock);
    std::ostringstream os;
    os << "Could not make SIOCGIFFLAGS ioctl : " << strerror(error); 
    throw std::runtime_error(os.str());
  }
  
  return (ifr.ifr_flags & IFF_UP);
}

// returns true if interface is running (has link)
bool is_iface_running(const char* iface_name) throw (std::exception)
{
  int request_sock = get_request_sock();
    
  // get interface flags
  struct ifreq ifr;                 
  strncpy(ifr.ifr_name, iface_name, IFNAMSIZ);
  if (ioctl(request_sock, SIOCGIFFLAGS, &ifr) < 0) {
    int error = errno;
    close(request_sock);
    std::ostringstream os;
    os << "Could not make SIOCGIFFLAGS ioctl : " << strerror(error); 
    throw std::runtime_error(os.str());
  }
  
  return (ifr.ifr_flags & IFF_RUNNING);
}



#define EXCEPT_HEADER  __FILE__ << ':' << __LINE__ << ' ' << __func__ << "  "

double timediff (const timespec &time1, const timespec &time0) 
{
  double diff = time1.tv_sec - time0.tv_sec;
  long long diff_ns = time1.tv_nsec - time0.tv_nsec;
  diff += ((double) diff_ns ) * 0.000000001;
  return diff;
}


// Use a thread data struct
struct send_thread {
  pthread_t thread;
  std::string name;
  unsigned thread_num;
  pthread_mutex_t thread_lock;
  bool running;

  uint64_t sent, interval_sent;
  uint64_t unsent, interval_unsent;
  uint64_t dropped, interval_dropped;   
  uint64_t good, interval_good;
  int interval_min_device_count;
  int max_device_count;
  struct timespec start_time, interval_time;

  unsigned packet_size;  
  unsigned char buffer[2000];
  unsigned buffer_index;
  unsigned char buffer_value;

  struct netif *ni;
  APRD_Telegram aprd_telegram;
  EC_Ethernet_Frame frame;

  static bool quit;
  static unsigned verbose;
  static pthread_mutex_t global_lock;
  static unsigned thread_count;


  static inline void lock_global() {
    pthread_mutex_lock(&global_lock);
  }

  static inline void unlock_global() {
    pthread_mutex_unlock(&global_lock);
  }

  inline void lock_thread() {
    pthread_mutex_lock(&thread_lock);
  }

  inline void unlock_thread() {
    pthread_mutex_unlock(&thread_lock);
  }


  send_thread(unsigned packet_size, struct netif *ni) : 
    running(false),
    sent(0), interval_sent(0),
    unsent(0), interval_unsent(0),
    dropped(0), interval_dropped(0),
    good(0), interval_good(0),
    interval_min_device_count(-1),
    max_device_count(-1),
    packet_size(packet_size),
    buffer_index(0),
    buffer_value(0xFF),
    ni(ni), 
    aprd_telegram(0,0,0,0,packet_size,buffer),
    frame(&aprd_telegram)
  {
    lock_global();
    ++thread_count;
    unlock_global();
    thread_num = thread_count;

    ostringstream os;
    os << "thread("<<thread_num<<")";
    name = os.str();
    
    if (verbose>0) {
      lock_global();
      cerr << " send_thread " << thread_count << " " << name << endl;
      unlock_global();    
    }

    int error = pthread_mutex_init(&thread_lock,NULL);
    if (error != 0) 
    {
      cerr << " error creating thread lock : " << strerror(error) << endl;  
    }
  }
  
  ~send_thread() {
    lock_global();
    --thread_count;
    if (verbose>0) {
      cerr << " ~send_thread " << thread_count << " " << name << endl;
    }
    if (running) {
      cout << " Error : destructor called on running thread" << endl;
    }
    unlock_global();
  }
  
  void start() throw(std::exception) {
    int error = pthread_create(&thread, NULL, this->run, this);
    if (error != 0) {
      ostringstream os;    
      os << EXCEPT_HEADER << "Error creating thread " << name << " : " << strerror(error);
      throw runtime_error(os.str());
    }
    running = true;
  }
  
  static void* run(void* arg) {
    send_thread *st = (send_thread *)arg;

    if (verbose>0) {
      st->lock_global();
      cerr << " run(1) " << st->name << endl;
      st->unlock_global();
    }
    st->run();    
    if (verbose>0) {
      st->lock_global(); 
      cerr << " ~run(1) " << st->name << endl;
      st->unlock_global(); 
    }
    st->running = false;
    return NULL;
  }
  
  void run() {
    if (verbose>0) {      
      lock_global();
      cerr << " run(2) " << name << endl;
      unlock_global();    
    }

    if (clock_gettime(CLOCK_REALTIME, &start_time) != 0) {
      int error = errno;
      fprintf(stderr, "%s: Could not get start time : %s\n",
             __func__, strerror(error));
      return;
    }
    interval_time = start_time;
    
    while (!quit) {
      int send_index = rand() & 0xFF;
      int send_wkc   = rand() & 0xFFFF;      
      
      aprd_telegram.set_adp(0);
      aprd_telegram.set_wkc(send_wkc);
      aprd_telegram.set_idx(send_index);
      
      // Put moving pattern in buffer
      buffer[buffer_index]=buffer_value;      
      ++buffer_index;
      if ((buffer_index>=sizeof(buffer)) || (buffer_index>=packet_size)) {
        buffer_index=0;
        buffer_value = ~buffer_value;
      }
      
      int handle = ni->tx(&frame, ni);
      if (handle<0) {
        lock_thread();
        ++unsent;
        ++interval_unsent;
        unlock_thread();
        if (verbose) {
          lock_global();
          cerr << '-';
          unlock_global();
        }
      } else {        
        lock_thread();
        ++sent;
        ++interval_sent;
        unlock_thread();
        
        bool success = ni->rx(&frame, ni, handle);

        lock_thread();
        if (!success) {

          ++dropped;
          ++interval_dropped;
        } else {
          ++good;
          ++interval_good;
          int device_count = aprd_telegram.get_adp();
          if ((interval_min_device_count == -1) || (device_count < interval_min_device_count)) 
          {
            // interval device count is minumum number of number of devices seen in a publishing interval.
            // -1 is a special case, it means that no packets have come back (hence device count cannnot be determined)
            interval_min_device_count = device_count;
          }
          if (device_count > max_device_count) 
          {
            max_device_count = device_count;
          }
        }
        unlock_thread();
        
        if (verbose) {
          lock_global();
          if (!success) {
            cerr << 'x';
          }
          else if (verbose>1) {
            cerr << '.';
          }
          unlock_global();
        }
      }
      
      if (send_index != aprd_telegram.get_idx()) {
        lock_global();
        cerr << " got invalid frame back : expected " << send_index 
             << " got " << (int)aprd_telegram.get_idx() << endl;
        unlock_global();
      }
    } // end while 
  }

  void collect_stats(ectools::ecstats &stats) 
  {
    struct timespec current_time;
    if (clock_gettime(CLOCK_REALTIME, &current_time) != 0) {
      int error = errno;
      fprintf(stderr,"%s: Could not get stoptime : %s\n",
             __func__, strerror(error));
      return;
    }
    
    double total_time_sec    = timediff(current_time, start_time);    
    double interval_time_sec = timediff(current_time, interval_time);
    static const unsigned ethernet_overhead = 14;
    static const unsigned ethercat_frame_overhead = 2;
    static const unsigned ethercat_datagram_overhead = 12;
    unsigned packet_size_bits = 8*(packet_size + ethernet_overhead + ethercat_frame_overhead + ethercat_datagram_overhead);    
    
    lock_thread();
    {      
      
      double mbits_per_bit = 1.0 / 1000000.0;
      stats.total_bandwidth_mbps    += mbits_per_bit * (double)(good * packet_size_bits) / total_time_sec;
      stats.interval_bandwidth_mbps += mbits_per_bit * (double)(interval_good * packet_size_bits) / interval_time_sec;
      
      stats.total_sent_packets += (sent + unsent);
      stats.interval_sent_packets += (interval_sent + interval_unsent);
      
      stats.total_dropped_packets += (dropped + unsent);
      stats.interval_dropped_packets += (interval_dropped + interval_unsent);
     
      if ((stats.interval_min_device_count == -1) ||
          ((interval_min_device_count!=-1) && (interval_min_device_count < stats.interval_min_device_count)))
      {
        stats.interval_min_device_count = interval_min_device_count;
      }
      
      if (max_device_count > stats.max_device_count)
      {
        stats.max_device_count = max_device_count;
      }   

      interval_sent = 0;
      interval_unsent = 0;
      interval_good = 0;
      interval_dropped = 0;
      interval_min_device_count = -1;
    }
    unlock_thread();
      
    interval_time = current_time;    
  }
  

  void print(std::ostream &os)
  {
    lock_thread();

    os << name 
       << " : attempts " <<  (good+dropped+unsent) 
       << " unsent " << unsent 
       << " dropped " << dropped 
       << " good " << good 
       << " max devices " << max_device_count 
       << " interval min devices " << interval_min_device_count
       << endl;

    unlock_thread();    
  }

};


pthread_mutex_t send_thread::global_lock = PTHREAD_MUTEX_INITIALIZER;
unsigned send_thread::thread_count = 0; 
unsigned send_thread::verbose = 0;
bool send_thread::quit = false;



int main(int argc, char** argv) {
  char const *progname = argv[0];
  if (argc < 2) {
    usage(progname);
    return 1;
  }

  ros::init(argc, argv, "ecstats");

  const char *interface = NULL; //default_interface;	
  unsigned packet_size = default_packet_size;
  unsigned num_threads = default_num_threads;  

  int optchar;
  int tmp;
  while ((optchar = getopt(argc, argv, "i:s:j:vh")) != -1) {		
    switch(optchar) { 
    case 'h':
      usage(progname);
      return 0;
      break;
    case 's':
      if ( sscanf(optarg, "%i", &tmp) != 1 ) {
        fprintf(stderr, "invalid packet size : '%s'\n", optarg);
        return 1;
      }
      if (tmp < 0) {
        fprintf(stderr, "size must be positive number\n");
        return 1;
      }
      packet_size = tmp;
      break;
    case 'j':
      if ( sscanf(optarg, "%i", &tmp) != 1 ) {
        fprintf(stderr, "invalid amount : '%s'\n", optarg);
        return 1;
      }
      if (tmp <= 0) {
        fprintf(stderr, "number of threads must be positive number\n");
        return 1;
      }
      num_threads = tmp;
      break;              
    case 'i': {
      interface = optarg;
      break;
    }
    case 'v':
      send_thread::verbose += 1;
      break;
    case '?':
      fprintf(stderr, "unknown/unsupported option %c\n", optopt);
      return 1;
    default:
      fprintf(stderr, "programming error %c = %d??\n", optchar, optchar);
      return 1;				
    } // end switch	   
  }

  if (interface == NULL) {
    fprintf(stderr,"You must specify and interface with (-i)\n");
    return 1;
  }

  // Must run as root
  if (geteuid() != 0)
  {
    fprintf(stderr, "You must run as root!\n");
    return 1;
  }
  
  if (!is_iface_up(interface)) {
    fprintf(stderr, "Interface %s is not UP, try 'ifconfig %s up'", interface, interface);
    return 1;
  }
    
  struct netif *ni = init_ec(interface);	 
  if (ni == NULL) {
    fprintf(stderr, "Initializing Network interface (%s) failed\n", interface);
    return 1;
  }

  if (set_socket_timeout(ni, 5000)) {
    printf("Setting socket timeout failed\n\n");		
    close_socket(ni);
    return 1;
  }

  ros::NodeHandle node;
  ros::Publisher ecstats_pub = node.advertise<ectools::ecstats>("ecstats", 100);
  ros::Rate loop_rate(5);
			 
  std::vector<send_thread*> thread_list;     

  for (unsigned i=0; i<num_threads; ++i) { 
    send_thread *p = new send_thread(packet_size, ni);
    if (p==NULL){ 
      throw std::bad_alloc();                
    }
    thread_list.push_back(p);
  }      
  
  BOOST_FOREACH( send_thread* t, thread_list ) {  
    assert(t!=NULL);
    t->start();
  }

  uint64_t late_packets_previous = 0;
  ectools::ecstats stats;
  stats.max_device_count = -1;
  while (ros::ok())
  {
    stats.has_link = is_iface_running(interface);
    stats.interval_min_device_count = -1;

    // Zero stats
    stats.total_sent_packets = 0;
    stats.total_dropped_packets = 0;
    stats.total_bandwidth_mbps = 0.0;
    stats.interval_sent_packets = 0;
    stats.interval_dropped_packets = 0;
    stats.interval_bandwidth_mbps = 0.0;

    // Accumulate data from each thread -- 
    BOOST_FOREACH( send_thread* &t, thread_list ) {  
      t->collect_stats(stats);
    }

    // Keep track of how many packets come in late because of operating system glitches
    uint64_t late_packets = ni->counters.rx_late_pkt;
    stats.total_late_packets = late_packets;
    stats.interval_late_packets = late_packets - late_packets_previous;
    late_packets_previous = late_packets;
   
    // Publish data
    ecstats_pub.publish(stats);

    ros::spinOnce();
    loop_rate.sleep();
  }

  
  cerr << "Quiting" << endl;
  send_thread::quit = true;
  usleep(10000);
  cout << endl;

  // Stop threads
  BOOST_FOREACH( send_thread* &t, thread_list ) {  
    t->print(cout);
    if (t!=NULL) {
      delete t;
      t = NULL;
    }
  }

  for (unsigned xx=0; xx<thread_list.size(); ++xx) {
    assert(thread_list[xx] == NULL);    
  }
    
  if (ni!=NULL) {
    close_socket(ni);
    ni = NULL;
  }

  return 0;
}
