/**
    @file
    @author  Matteo Ciocca

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

namespace humoto
{
    namespace wpg04
    {
        /// @brief Class used to log and display the CoM and CoP history
        class HUMOTO_LOCAL Logger
        {
            private:
                // Difference between this and just "Eigen::Vector3d":
                // For now I know "std::vector<Eigen::Vector3d>" does not have the function of Eigen
                std::vector<Eigen::Vector3d> positionsCoM_;
                std::vector<Eigen::Vector3d> positionsCoP_;
                std::vector<Eigen::Vector3d> velocitiesCoM_;
                std::vector<Eigen::Vector3d> accelerationsCoM_;
                std::vector<Eigen::Vector3d> jerksCoM_;
                Eigen::MatrixXd              positionObject_;

                Eigen::VectorXd xMin_;
                Eigen::VectorXd xMax_;
                Eigen::VectorXd yMin_;
                Eigen::VectorXd yMax_;
                Eigen::VectorXd zMin_;
                Eigen::VectorXd zMax_;
                double timeStep_;


            public:
                /// @brief Constructor
                // Logger(double timeStep, const StepPlan& stepPlan) : timeStep_(timeStep)
                // {
                //     xMin_ = stepPlan.xMin();
                //     xMax_ = stepPlan.xMax();
                //     yMin_ = stepPlan.yMin();
                //     yMax_ = stepPlan.yMax();
                //     zMin_ = stepPlan.zMin();
                //     zMax_ = stepPlan.zMax();
                // }


                Logger(double timeStep) : timeStep_(timeStep)
                {
                }


                /// @brief Adds a single state and control to the history
                ///
                /// @param state state
                /// @param control control
                void addStateAndControl(const etools::Vector9& state,
                                        double h_CoM,
                                        double g,
                                        const std::vector<boost::shared_ptr<humoto::obstacle_avoidance::ObstacleBase> >& obstacles)
                {
                    Eigen::Vector3d position, velocity, acceleration, cop;

                    position << state(0), state(3), state(6);
                    velocity << state(1), state(4), state(7);
                    acceleration << state(2), state(5), state(8);
                    positionsCoM_.push_back(position);
                    velocitiesCoM_.push_back(velocity);
                    accelerationsCoM_.push_back(acceleration);
                    cop(0) = position(0) - h_CoM / g * acceleration(0);
                    cop(1) = position(1) - h_CoM / g * acceleration(1);
                    cop(2) = 0;
                    positionsCoP_.push_back(cop);

                    if(!obstacles.empty())
                    {
                        positionObject_.resize(2, obstacles.size());
                        for(std::size_t i = 0; i < obstacles.size(); ++i)
                        {
                            positionObject_.block(0, i, 2, 1) << obstacles[i]->position_.segment(0, 2); 
                        }
                    }
                }


                Eigen::MatrixXd getPositionsAsMatrix() const { return toMatrix(positionsCoM_); }
                Eigen::MatrixXd getVelocitiesAsMatrix() const { return toMatrix(velocitiesCoM_); }
                Eigen::MatrixXd getAccelerationsAsMatrix() const { return toMatrix(accelerationsCoM_); }
                Eigen::MatrixXd getJerksAsMatrix() const { return toMatrix(jerksCoM_); }
                Eigen::MatrixXd getCoPsAsMatrix() const { return toMatrix(positionsCoP_); }


                /// @brief Transforms a list of vector3 into a matrix
                ///
                /// @param vec list of vector3
                ///
                /// @return Matrix concatenation of the vectors
                Eigen::MatrixXd toMatrix(const std::vector<Eigen::Vector3d>& vec) const
                {
                    Eigen::MatrixXd mat;
                    mat.resize(vec.size(), 3);
                    for (size_t i = 0; i < vec.size(); ++i) mat.row(i) << vec[i].transpose();
                    return mat;
                }


                /// @brief Getter for size
                size_t size() const { return positionsCoM_.size(); }

                /// @brief Prints the history of the CoM
                void print() const
                {
                    std::cout << "Position History:" << std::endl;
                    std::cout << toMatrix(positionsCoM_) << std::endl;
                    std::cout << "Velocity History:" << std::endl;
                    std::cout << toMatrix(velocitiesCoM_) << std::endl;
                    std::cout << "Acceleration History:" << std::endl;
                    std::cout << toMatrix(accelerationsCoM_) << std::endl;
                    std::cout << "Jerk History:" << std::endl;
                    std::cout << toMatrix(jerksCoM_) << std::endl;
                }

                /// @brief Writes a python file that can later be executed to display the history of the system
                void plot() const
                {
                    Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
                    std::ofstream logFile("plotFile.py");
                    logFile << "import numpy as np\n"
                            << "import matplotlib.pyplot as plt\n"
                            << "t = np.arange(0.," << (double)size() * (double)timeStep_ << ", " << timeStep_ << ")\n\n";

                    Eigen::MatrixXd positions(toMatrix(positionsCoM_));
                    Eigen::MatrixXd positionsCoP(toMatrix(positionsCoP_));
                    Eigen::MatrixXd velocities(toMatrix(velocitiesCoM_));
                    Eigen::MatrixXd accelerations(toMatrix(accelerationsCoM_));
                    // Eigen::MatrixXd jerks(toMatrix(jerksCoM_));

                    logFile << "x = np.array(" << positions.col(0).transpose().format(cleanFmt) << ")\n";
                    logFile << "y = np.array(" << positions.col(1).transpose().format(cleanFmt) << ")\n";
                    logFile << "z = np.array(" << positions.col(2).transpose().format(cleanFmt) << ")\n";

                    logFile << "xCoP = np.array(" << positionsCoP.col(0).transpose().format(cleanFmt) << ")\n";
                    logFile << "yCoP = np.array(" << positionsCoP.col(1).transpose().format(cleanFmt) << ")\n";

                    // position in the space of the object
                    if(positionObject_.size())
                    {   
                        for(std::size_t i = 0; i < static_cast<std::size_t>(positionObject_.cols()); ++i)
                        {
                            logFile << "xObj = np.array(" << positionObject_.row(0).format(cleanFmt) << ")\n";
                            logFile << "yObj = np.array(" << positionObject_.row(1).format(cleanFmt) << ")\n";
                        }
                    }
                    
                    // logFile << "xMin = np.array(" << xMin_.transpose().format(cleanFmt) << ")\n";
                    // logFile << "yMin = np.array(" << yMin_.transpose().format(cleanFmt) << ")\n";
                    // logFile << "zMin = np.array(" << zMin_.transpose().format(cleanFmt) << ")\n";
                    // logFile << "xMax = np.array(" << xMax_.transpose().format(cleanFmt) << ")\n";
                    // logFile << "yMax = np.array(" << yMax_.transpose().format(cleanFmt) << ")\n";
                    // logFile << "zMax = np.array(" << zMax_.transpose().format(cleanFmt) << ")\n";

                    logFile << "dx = np.array(" << velocities.col(0).transpose().format(cleanFmt) << ")\n";
                    logFile << "dy = np.array(" << velocities.col(1).transpose().format(cleanFmt) << ")\n";
                    logFile << "dz = np.array(" << velocities.col(2).transpose().format(cleanFmt) << ")\n";

                    logFile << "ddx = np.array(" << accelerations.col(0).transpose().format(cleanFmt) << ")\n";
                    logFile << "ddy = np.array(" << accelerations.col(1).transpose().format(cleanFmt) << ")\n";
                    logFile << "ddz = np.array(" << accelerations.col(2).transpose().format(cleanFmt) << ")\n";

                    // logFile << "dddx = np.array(" << jerks.col(0).transpose().format(cleanFmt) << ")\n";
                    // logFile << "dddy = np.array(" << jerks.col(1).transpose().format(cleanFmt) << ")\n";
                    // logFile << "dddz = np.array(" << jerks.col(2).transpose().format(cleanFmt) << ")\n";


                    //--------------------- Create the Plot --------------------------------------


                    // logFile << "f, (ax1, ax5) = plt.subplots(2, sharex=False, sharey=False)\n";
                    logFile << "f, (ax1, ax5, ax7) = plt.subplots(3, sharex=False, sharey=False)\n";

                    logFile << "ax1.plot(t, x, 'r', label='x')\n";
                    logFile << "ax1.plot(t, y, 'b', label='y')\n";
                    logFile << "ax1.plot(t, z, 'g', label='z')\n";
                    logFile << "ax1.plot(t, xCoP, 'ro', label='xCoP')\n";
                    logFile << "ax1.plot(t, yCoP, 'bo', label='yCoP')\n";
                    logFile << "ax1.set_xlabel('Time (s)')\n";
                    logFile << "ax1.set_ylabel('positions (m)')\n";
                    logFile << "ax1.set_ylim(-0.5,1.8)\n";
                    logFile << "ax1.set_title('Positions')\n";
                    logFile << "legend = ax1.legend(loc='upper center', shadow=True)\n";


                    // logFile << "ax5.plot(x, y, 'r', label='traj CoM')\n";
                    // logFile << "ax5.plot(xCoP, yCoP, 'b', label='traj CoP')\n";
                    // logFile << "ax5.set_title('Trajectories')\n";
                    // logFile << "ax5.set_xlabel('position x (m)')\n";
                    // logFile << "ax5.set_ylabel('position y (m)')\n";
                    // logFile << "legend = ax5.legend(loc='upper center', shadow=True)\n";

                    logFile << "ax5.plot(x, y, 'r', label='traj CoM')\n";
                    logFile << "ax5.plot(xCoP, yCoP, 'b', label='traj CoP')\n";

                    if(positionObject_.size())
                    {    
                        for(std::size_t i = 0; i < static_cast<std::size_t>(positionObject_.cols()); ++i)
                        {
                            // logFile << "ax5.plot(xObj, yObj, 'g', label='pos Obj')\n";
                            logFile << "ax5.plot(xObj" << "[" << i << "]" << " ," <<
                                " yObj" << "[" << i << "]" << ", 'g', alpha=0.5, marker='o', markersize=20)\n";
                        }
                    }
                    
                    logFile << "ax5.set_title('Trajectories')\n";
                    logFile << "ax5.set_xlabel('position x (m)')\n";
                    logFile << "ax5.set_ylabel('position y (m)')\n";
                    logFile << "legend = ax5.legend(loc='upper center', shadow=True)\n";


                    logFile << "ax7.plot(t, dx, 'r', label='dx CoM')\n";
                    logFile << "ax7.plot(t, dy, 'b', label='dy CoM')\n";
                    logFile << "ax7.plot(t, dz, 'g', label='dz CoM')\n";
                    logFile << "ax7.set_title('Velocties')\n";
                    logFile << "ax7.set_xlabel('velocity (m/s)')\n";
                    logFile << "ax7.set_ylabel('time (s)')\n";
                    logFile << "legend = ax7.legend(loc='upper center', shadow=True)\n";

                    logFile << "plt.axis('auto')\n";
                    logFile << "plt.show()\n";
                    logFile.close();
                }


        };
    }//wpg04
}//humoto
