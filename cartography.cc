#include "Aria.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    // Create a file in which we store the x, y, theta in odometric positioning
    std::ofstream file;
    file.open ("test.txt");

    // Initialization
    Aria::init();
    ArArgumentParser argParser(&argc, argv);
    argParser.loadDefaultArguments();
    // Create a robot
    ArRobot robot;
    ArRobotConnector robotConnector(&argParser, &robot);
    // Add a laser to the robot
    ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

    argParser.addDefaultArgument("-connectLaser");

    // Connect to the robot in MobileSim
    if(!robotConnector.connectRobot())
    {
        // Error handling
        ArLog::log(ArLog::Terse, "Could not connect to the robot.");
        if(argParser.checkHelpAndWarnUnparsed())
        {
            Aria::logOptions();
            Aria::exit(1);
        }
    }

    // Parsing handling
    if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }

    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);

    // Add a sonar
    ArSonarDevice sonar;
    robot.addRangeDevice(&sonar);

    // Run the thread that will execute all the actions that will be defined below
    robot.runAsync(true);

    // Check if the laser connectors is working correctly
    if(!laserConnector.connectLasers())
    {
        ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
    }

    // Enable the robot motors and remove the sound
    robot.enableMotors();
    robot.comInt(ArCommands::SOUNDTOG, 0);

    // Add collision detection using predefined Aria classes
    // The robot can now go straight and avoid walls on collision
    ArActionStallRecover recover;
    ArActionBumpers bumpers;
    ArActionAvoidFront avoidFrontNear("Avoid Front Near", 225, 0);
    ArActionAvoidFront avoidFrontFar;
    ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
    // Add the actions to the robot
    robot.addAction(&recover, 100);
    robot.addAction(&bumpers, 75);
    robot.addAction(&avoidFrontNear, 50);
    robot.addAction(&avoidFrontFar, 49);
    robot.addAction(&constantVelocity, 25);

    // Tell the robot to log the laser data
    laserConnector.logLaserData();
    ArUtil::sleep(500);

    // Loop to save all the laser data while the robot moves
    while(robot.isConnected())
    {
        int numLasers = 0;
        // Lock the robot to avoid problems with the other running thread
        robot.lock();
        std::map<int, ArLaser*> *lasers = robot.getLaserMap();
        // Iterate over all lasers
        for(std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
        {
            int laserIndex = (*i).first;
            ArLaser* laser = (*i).second;
            if(!laser)
                continue;
            ++numLasers;
            // Lock the laser so that the robot doesn't change the data while we read it
            laser->lockDevice();
            // Get all the current laser data
            // The data changes each 3000ms
            std::list<ArPoseWithTime*> *currentReadings = laser->getCurrentBuffer();

            for (auto i : *currentReadings)
                file << i->getX() << " " << i->getY() << " " << i->getTh() << std::endl;
            std::cout << "Laser " << laserIndex << " " << robot.getX() << " " << robot.getY() << " " << robot.getTh() << std::endl;

            // Unlock the laser so that the robot can use it
            laser->unlockDevice();
        }
        // Handle errors
        if(numLasers == 0)
            ArLog::log(ArLog::Normal, "No lasers found.");
        else
            ArLog::log(ArLog::Normal, "");
        // Unlock the robot so that the other thread can use it for the default actions
        robot.unlock();
        // Wait 3000ms
        ArUtil::sleep(3000);
    }

    // Close the file with data
    file.close();

    // Clean Aria and exit
    Aria::exit(0);
}
