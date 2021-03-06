/**
    @file
    @author  Alexander Sherikov
    @author  Jan Michalczyk
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <limits>
#include <iomanip>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
// common & abstract classes
#include "humoto/humoto.h"
// specific solver (many can be included simultaneously)
#include "humoto/qpoases.h"
// obstacle avoidance
#include "humoto/obstacle_avoidance.h"
// specific control problem (many can be included simultaneously)
#include "humoto/wpg04.h"

#include "utilities.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);
//HUMOTO_INITIALIZE_GLOBAL_LOGGER("test_000.m");

/**
 * @brief Simple control loop
 *
 * @param[in] argc number of arguments
 * @param[in] argv arguments
 *
 * @return status
 */
int main(int argc, char **argv)
{
    std::string log_file_name = humoto_tests::getLogFileName(argc, argv);

    try
    {
        std::string config_file_name = std::string(argv[0]) + ".yaml";

        humoto::config::yaml::Reader config_reader(config_file_name);

        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem               opt_problem;

        // parameters of the solver
        humoto::qpoases::SolverParameters         solver_parameters(config_reader);
        // a solver which is giong to be used
        humoto::qpoases::Solver                   solver(solver_parameters);

        // solution
        humoto::qpoases::Solution                 solution;

        // options for walking
        humoto::wpg04::WalkParameters             walk_parameters(config_reader);
        /* Or set parameters manually
        walk_parameters.com_velocity_ << 0.1, 0.;
        walk_parameters.first_stance_com_velocity_ = walk_parameters.com_velocity_;
        walk_parameters.num_steps_ = 7;
        */

        // FSM for walking
        humoto::walking::StanceFiniteStateMachine stance_fsm(walk_parameters);

        // model representing the controlled system
        humoto::wpg04::Model                      model;

        // initialize obstacles
        etools::Vector3 obstacle_position;
        etools::Vector3 obstacle_rpy;
        obstacle_position << 0.3, 0.05, 0.0;
        obstacle_rpy      << 0.0, 0.0,  0.0;


        model.addObstacle(boost::shared_ptr<humoto::obstacle_avoidance::ObstacleBase>(new
                                humoto::wpg04::ObstacleCircle(obstacle_position, obstacle_rpy)));

        obstacle_position << 0.6, -0.1, 0.0;
        obstacle_rpy      << 0.0, 0.0,  0.0;
        model.addObstacle(boost::shared_ptr<humoto::obstacle_avoidance::ObstacleBase>(new
                                humoto::wpg04::ObstacleCircle(obstacle_position, obstacle_rpy)));



        // safety margin for obstacle avoidance
        double safety_margin = 0.1;

        // parameters of the control problem
        humoto::wpg04::MPCParameters              wpg_parameters(config_reader);
        // control problem, which is used to construct an optimization problem
        humoto::wpg04::MPCforWPG                  wpg(wpg_parameters);

        humoto::wpg04::Logger mylogger(wpg_parameters.getSubsamplingTime());

        // ACTIVE SET GUESSING
        humoto::ActiveSet active_set_guess;
        humoto::ActiveSet active_set_actual;
        // -----

        // SOLUTION GUESSING
        humoto::qpoases::Solution   old_solution;
        humoto::Solution            solution_guess;
        // -----

        // Hierarchy with Collision Avoidance Constraints
        setupHierarchy_v_mine(opt_problem);

        humoto::wpg04::ModelState model_state;

        humoto::Timer        timer;
        humoto::Logger       logger(log_file_name);
        humoto::LogEntryName prefix;

        for (std::size_t i = 0;; ++i)
        {
            prefix = humoto::LogEntryName("humoto").add(i);

            timer.start();

            // -------------------------------------------------
            // I.   prepare control problem for new iteration
            if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }
            // -------------------------------------------------

            // -------------------------------------------------
            // in each pass of the loop vector with obstacles should be cleared
            // and new detected obstacles should be added to it and then
            // these new obstacles should be updated using updateObstacles()

            // Update obstacles in the model - after updating control problem
            model.updateObstacles(wpg, i, old_solution, safety_margin);
            // -------------------------------------------------

            // -------------------------------------------------
            // II.  form an optimization problem
            //  1. no guessing of solution or active set
            //opt_problem.form(solution, model, wpg);
            //
            //  2. if you are using active set guessing only
            //opt_problem.form(solution, active_set_guess, model, wpg);
            //
            //  3. if you are using active set guessing and solution guessing
            opt_problem.form(solution, solution_guess, active_set_guess, model, wpg, old_solution);
            // -------------------------------------------------

            // -------------------------------------------------
            // III. solve an optimization problem
            //
            //  1. no guessing of solution or active set
            //solver.solve(solution, opt_problem);
            //
            //  2. if you are using active set guessing only
            //solver.solve(solution, active_set_actual, opt_problem, active_set_guess);
            //opt_problem.processActiveSet(active_set_actual);
            //
            //  3. if you are using active set guessing and solution guessing
            solver.solve(solution, active_set_actual, opt_problem, solution_guess, active_set_guess);
            opt_problem.processActiveSet(active_set_actual);
            old_solution = solution;
            // -------------------------------------------------

            //========================
            // logging (optional)
            opt_problem.log(logger, prefix);
            solver.log(logger, prefix);
            solution.log(logger, prefix);
            stance_fsm.log(logger, prefix);
            wpg.log(logger, prefix);
            model.log(logger, prefix);

            active_set_guess.log(logger, prefix, "active_set_guess");
            active_set_actual.log(logger, prefix, "active_set_actual");

            solution_guess.log(logger, prefix, "solution_guess");
            old_solution.log(logger, prefix, "old_solution");
            //========================

            // -------------------------------------------------
            // IV.  extract next model state from the solution and update model
            stance_fsm.shiftTime(wpg_parameters.sampling_time_ms_);
            model_state = wpg.getNextModelState(solution, stance_fsm, model, mylogger);
            model.updateState(model_state);
            // -------------------------------------------------

            timer.stop();
            HUMOTO_LOG_RAW(timer);
        }

        // Create the plot file (.py)
        mylogger.plot();

        // Execute the command { python plotFile.py } in terminal
        std::string command = "python plotFile.py";
        system(command.c_str());

    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }
    return (0);
}
