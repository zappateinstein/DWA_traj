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
float obs_coords[][2] = {
    // 1. Un mur vertical en X=1.5 (Force le détour)
    {1.5f, -1.0f}, {1.5f, 0.0f}, {1.5f, 1.0f}, {1.5f, 2.0f},
    
    // 2. Un mur horizontal en Y=3.0 (Barrière du haut)
    {-1.0f, 3.0f}, {0.0f, 3.0f}, {1.0f, 3.0f},
    
    // 3. Un "Goulot" étroit en (3.5, 2.0)
    {-3.5f, 1.5f}, {3.5f, 3.5f}, 
    
    // 4. Protection autour du but (supposé vers 5,5)
    {4.5f, 4.0f}, {-4.0f, 5.5f},
    
    // 5. Obstacles dispersés (Pièges)
    {2.0f, 2.0f}, {2.0f, 4.0f}, {2.5f, -1.5f}, {5.0f, 0.0f}
};
// Nombre total d'obstacles dans la liste ci-dessus
int nb_obs = 15; 
// =========================================================

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
  Man* man;
#endif

  parseOptions(argc, argv);

  simu = new Simulator("simulator", opti_time, 90);
  simu->SetupConnection(address, port);
  simu->SetupUserInterface(xml_file);

#ifdef GL
  gui = new Parser(960, 480, 960, 480, media_path, scene_file);
#endif
  
  robot = new TwoWheelRobot(name, 0);

#ifdef GL
  man = new Man("target", 1);

  // CRÉATION DES BALLES DANS LE SIMULATEUR
  for(int i = 0; i < nb_obs; i++) {
      std::stringstream ss;
      ss << "obs_" << i;
      new Ball(ss.str(), 10 + i);
  }
#endif

#ifdef GL
  gui->setVisualizationCamera(robot);
#endif
  
  simu->RunSimu();

  delete simu;
  return 0;
}