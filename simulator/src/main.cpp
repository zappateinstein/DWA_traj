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
#include <vector>  // <--- AJOUTER
#include <string>  // <--- AJOUTER
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


void parseOptions(int argc, char** argv)
{
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

    // Get the value parsed by each arg.
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
  man = new Man("target",1);
#endif
  gui->setVisualizationCamera(robot);
  simu->RunSimu();

  delete simu;

  return 0;
}

