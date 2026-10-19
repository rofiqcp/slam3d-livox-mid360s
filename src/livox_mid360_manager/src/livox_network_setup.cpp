#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ifaddrs.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>

#include <iostream>
#include <stdexcept>
#include <string>

#include "livox_mid360_manager/yaml_utils.hpp"

static bool interface_has_ip(const std::string &iface, const std::string &ip, int prefix) {
  ifaddrs *ifaddr = nullptr;
  if (getifaddrs(&ifaddr) != 0) return false;

  bool found = false;
  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
    if (iface != ifa->ifa_name) continue;

    char buf[INET_ADDRSTRLEN] = {};
    auto *addr = reinterpret_cast<sockaddr_in *>(ifa->ifa_addr);
    inet_ntop(AF_INET, &addr->sin_addr, buf, sizeof(buf));
    const auto *mask = reinterpret_cast<sockaddr_in *>(ifa->ifa_netmask);
    const uint32_t expected = prefix == 0 ? 0 : (0xffffffffu << (32-prefix));
    if (ip == buf && mask && ntohl(mask->sin_addr.s_addr) == expected) {
      found = true;
      break;
    }
  }
  freeifaddrs(ifaddr);
  return found;
}

static int run_ip_add(const std::string &iface, const std::string &cidr) {
  pid_t pid = fork();
  if (pid == 0) {
    execlp("ip", "ip", "addr", "add", cidr.c_str(), "dev", iface.c_str(),
           static_cast<char *>(nullptr));
    _exit(127);
  }

  int status = 0;
  if (pid < 0 || waitpid(pid, &status, 0) < 0) return 1;
  return WIFEXITED(status) ? WEXITSTATUS(status) : 1;
}

int main(int argc, char **argv) {
  std::string config;
  std::string iface;
  std::string host_ip;
  int prefix = 24;
  bool apply = false, prefix_explicit = false;

  const char *explicit_config = std::getenv("LIVOX_MID360_CONFIG");
  const char *xdg = std::getenv("XDG_CONFIG_HOME");
  const char *home = std::getenv("HOME");
  if (explicit_config && *explicit_config) {
    config = explicit_config;
  } else if (xdg && *xdg) {
    config = std::string(xdg) + "/livox_mid360_manager/livox_runtime.yaml";
  } else if (home && *home) {
    config = std::string(home) + "/.config/livox_mid360_manager/livox_runtime.yaml";
  } else {
    config = "livox_runtime.yaml";
  }
  {
    std::ifstream probe(config);
    if (!probe.good()) {
      try {
        config = ament_index_cpp::get_package_share_directory("livox_mid360_manager") +
                 "/config/livox_runtime.yaml";
      } catch (...) {
      }
    }
  }

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char *opt) {
      if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + opt);
      return std::string(argv[++i]);
    };

    if (a == "--config") config = need("--config");
    else if (a == "--interface") iface = need("--interface");
    else if (a == "--host-ip") host_ip = need("--host-ip");
    else if (a == "--prefix") { prefix = std::stoi(need("--prefix")); prefix_explicit = true; }
    else if (a == "--apply") apply = true;
    else if (a == "-h" || a == "--help") {
      std::cout << "Usage: livox_network_setup [--config YAML] [--interface IFACE] "
                   "[--host-ip IP] [--prefix N] [--apply]\n";
      return 0;
    }
  }

  if (iface.empty()) {
    iface = livox_mid360_manager::yaml_scalar(config, "network", "interface").value_or("");
  }
  if (host_ip.empty()) {
    host_ip = livox_mid360_manager::yaml_scalar(config, "network", "host_ip").value_or("");
  }
  if (!prefix_explicit) if (auto p = livox_mid360_manager::yaml_scalar(config, "network", "prefix_length")) {
    prefix = std::stoi(*p);
  }

  if (iface.empty() || host_ip.empty()) {
    std::cerr << "Missing interface/host_ip. Set them in YAML or pass arguments.\n";
    return 2;
  }

  in_addr address{};
  if (prefix < 0 || prefix > 32 || inet_pton(AF_INET, host_ip.c_str(), &address) != 1) {
    std::cerr << "Invalid IPv4 address or prefix [0,32]\n"; return 2;
  }
  if (interface_has_ip(iface, host_ip, prefix)) {
    std::cout << "OK: " << iface << " already owns " << host_ip << "/" << prefix << "\n";
    return 0;
  }

  const std::string cidr = host_ip + "/" + std::to_string(prefix);
  if (!apply) {
    std::cout << "Not configured: " << iface << " lacks " << cidr << "\n"
              << "Apply with:\n"
              << "  sudo -E ros2 run livox_mid360_manager livox_network_setup "
              << "--config " << config << " --apply\n";
    return 3;
  }

  if (geteuid() != 0) {
    std::cerr << "--apply requires root. Re-run with sudo -E.\n";
    return 4;
  }

  const int rc = run_ip_add(iface, cidr);
  if (rc != 0) {
    std::cerr << "Failed to add " << cidr << " to " << iface
              << " (ip exit " << rc << ")\n";
    return 5;
  }

  std::cout << "Applied " << cidr << " to " << iface << "\n";
  return 0;
}
