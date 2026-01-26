//  created:    2020/11/20
//  filename:   main.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 6599
//
//  version:    $Id: $
//
//  purpose:    main simulateur
//
//
/*********************************************************************/

#include <tclap/CmdLine.h>
#include <Simulator.h>
#include <TwoWheelRobot.h>
#ifdef GL
#include <Parser.h>
#include <Man.h>
#include <Ball.h>
#include <vector>
#include <string>
#include <sstream>
#endif
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>

using namespace TCLAP;
using namespace std;
using namespace flair::simulator;
using namespace flair::sensor;

int port;
int opti_time;
string xml_file;
string media_path;
string scene_file;
string name;
string address;

// =========================================================
// LISTE MANUELLE D'OBSTACLES (SCÉNARIO COMPLEXE)
// =========================================================
float obs_coords_ctrl[][2] = { {-4.2f, -3.8f}
};
// Nombre total d'obstacles (sera mis à jour via UDP)
std::atomic<int> nb_obs(0);  // Commence à 0 au lieu de 1
std::atomic<bool> nb_obs_received(false);
std::atomic<int> nb_obs_created(0);  // Nombre d'obstacles actuellement créés
std::atomic<bool> need_recreate_obstacles(false);  // Flag pour signaler qu'il faut recréer
// =========================================================

// Thread UDP pour recevoir nb_obstacles depuis l'UGV
void udpReceiverThread(int udp_port) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "[Simulator] Failed to create UDP socket\n";
        return;
    }
    
    // Rendre le socket non-bloquant
    fcntl(sockfd, F_SETFL, O_NONBLOCK);
    
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(udp_port);
    
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        std::cerr << "[Simulator] Failed to bind UDP socket on port " << udp_port << "\n";
        close(sockfd);
        return;
    }
    
    std::cerr << "[Simulator] UDP listener started on port " << udp_port << "\n";
    
    char buffer[1024];
    while (true) {
      memset(buffer, 0, sizeof(buffer));
      int n = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
        
      if (n > 0) {
        buffer[n] = '\0';
        std::string msg(buffer);
    
        // ========== NOUVEAU : Parser message INIT ==========
        if (msg.substr(0, 4) == "INIT") {
          std::istringstream ss(msg);
          std::string header;
          std::getline(ss, header, ',');  // Skip "INIT"
        
          std::string nb_str;
          if (std::getline(ss, nb_str, ',')) {
            int received_nb_obs = std::stoi(nb_str);
            if (!nb_obs_received.load()) {
              nb_obs.store(received_nb_obs);
              nb_obs_received.store(true);
              std::cerr << "[Simulator] Received INIT nb_obstacles=" 
                        << received_nb_obs << " from UGV\n";
            }
          }
        }
        // ===================================================
        
        // Parser le message de télémétrie DWA (reste inchangé)
        else if (msg.substr(0, 3) == "DWA") {
          std::istringstream ss(msg);
          std::string token;
          int field_index = 0;
          
          while (std::getline(ss, token, ',')) {
            if (field_index == 9) {
              int received_nb_obs = std::stoi(token);
              if (nb_obs.load() != received_nb_obs) {
                nb_obs.store(received_nb_obs);
                nb_obs_received.store(true);
                need_recreate_obstacles.store(true);
                std::cerr << "[Simulator] Received nb_obstacles=" 
                          << received_nb_obs << " from UGV\n";
              }
              break;
            }
            field_index++;
          }
        }
      }
        
        usleep(10000);  // 10ms sleep pour éviter de saturer le CPU
    }
    
    close(sockfd);
}

void parseOptions(int argc, char** argv) {
  try {
    CmdLine cmd("Command description message", ' ', "0.1");
    ValueArg<string> nameArg("n", "name", "uav name, also used for vrpn", true, "x4", "string");
    cmd.add(nameArg);
    ValueArg<string> xmlArg("x", "xml", "xml file", true, "./reglages.xml", "string");
    cmd.add(xmlArg);
    ValueArg<int> portArg("p", "port", "ground station port", true, 9002, "int");
    cmd.add(portArg);
    ValueArg<string> addressArg("a", "address", "ground station address", true, "127.0.0.1", "string");
    cmd.add(addressArg);
    ValueArg<int> optiArg("o", "opti", "optitrack time ms", false, 0, "int");
    cmd.add(optiArg);
#ifdef GL
    ValueArg<string> mediaArg("m", "media", "path to media files", true, "./", "string");
    cmd.add(mediaArg);
    ValueArg<string> sceneArg("s", "scene", "path to scene file", true, "./voliere.xml", "string");
    cmd.add(sceneArg);
#endif
    cmd.parse(argc, argv);
    port = portArg.getValue();
    xml_file = xmlArg.getValue();
    opti_time = optiArg.getValue();
    name = nameArg.getValue();
    address = addressArg.getValue();
#ifdef GL
    media_path = mediaArg.getValue();
    scene_file = sceneArg.getValue();
#endif
  } catch(ArgException& e) {
    cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {
  Simulator* simu;
  Model* robot;
#ifdef GL
  Parser* gui;
#endif

  parseOptions(argc, argv);

  simu = new Simulator("simulator", opti_time, 90);
  simu->SetupConnection(address, port);
  simu->SetupUserInterface(xml_file);

#ifdef GL
  gui = new Parser(960, 480, 960, 480, media_path, scene_file);
#endif
  
  robot = new TwoWheelRobot(name, 0);

  // ========== DÉMARRAGE DU THREAD UDP ==========
  // Le port 9005 correspond au port utilisé par l'UGV dans dwatraj.cpp
  std::thread udpThread(udpReceiverThread, 9005);
  udpThread.detach();  // Détacher pour qu'il s'exécute en arrière-plan
  
  std::cerr << "[Simulator] Waiting for nb_obstacles from UGV (max 60 seconds)...\n";
  
  // Attendre jusqu'à 60 secondes pour recevoir nb_obstacles
  int wait_count = 0;
  while (!nb_obs_received.load() && wait_count < 600) {  // 600 * 100ms = 60 secondes
    usleep(100000);  // 100ms
    wait_count++;
  }

  int desired_nb_obs = nb_obs.load();
  std::cerr << "[Simulator] Creating " << desired_nb_obs << " obstacles\n";

#ifdef GL
  gui->setVisualizationCamera(robot);
  
  // Créer SEULEMENT le nombre d'obstacles reçu de l'UGV
  for(int i = 0; i < desired_nb_obs; i++) {
      std::stringstream ss;
      ss << "obs_" << i;
      new Ball(ss.str(), 10 + i);
  }
  
  std::cerr << "[Simulator] Created " << desired_nb_obs << " obstacle(s)\n";
#endif
  
  simu->RunSimu();

  delete simu;
  return 0;
}